"""Hardware-free reset and serial regression tests using a real ROS executor."""
# ruff: noqa: ANN201 D102 SLF001 PLR2004
import sys
import time
import unittest
from collections.abc import Callable
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

import rclpy
import serial
from offboard_comms.arduino_reset import ArduinoReset
from offboard_comms.serial_node import SerialNode
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger

CONFIG = {'pins': {'thruster': 29, 'peripheral': 31}, 'pulse_seconds': 0.1, 'cooldown_seconds': 3}


class ResetTests(unittest.TestCase):
    """Exercise timer completion and failures with all GPIO operations mocked."""

    def setUp(self):
        rclpy.init()
        self.node = Node('reset_test')
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.gpio = Mock(BOARD=10, OUT=1, LOW=0, HIGH=1)
        self.modules = patch.dict(sys.modules, {'Jetson': SimpleNamespace(GPIO=self.gpio)})
        self.modules.start()
        self.helpers = []

    def tearDown(self):
        for helper in self.helpers:
            helper.close()
        self.executor.shutdown()
        self.node.destroy_node()
        self.modules.stop()
        rclpy.shutdown()

    def helper(self, board: str = 'thruster'):
        helper = ArduinoReset(self.node, board, CONFIG)
        self.helpers.append(helper)
        return helper

    def spin_until(self, predicate: Callable[[], bool]):
        deadline = time.monotonic() + 2
        while not predicate() and time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.01)
        assert predicate()

    def test_pulse_overlap_cooldown_and_callback_progress(self):
        helper = self.helper()
        self.gpio.setup.assert_called_once_with(29, 1, initial=0)
        self.gpio.output.assert_not_called()
        client = self.node.create_client(Trigger, '/offboard/reset_thruster')
        assert client.wait_for_service(timeout_sec=1)
        ticks = []
        self.node.create_timer(0.01, lambda: ticks.append(time.monotonic()))
        started = time.monotonic()
        first = client.call_async(Trigger.Request())
        self.spin_until(lambda: helper._pending is not None)
        overlap = client.call_async(Trigger.Request())
        self.spin_until(overlap.done)
        assert not overlap.result().success
        assert 'in progress' in overlap.result().message
        assert not first.done()
        self.spin_until(first.done)
        assert first.result().success
        assert time.monotonic() - started >= 0.1
        assert len(ticks) > 1
        cooldown = client.call_async(Trigger.Request())
        self.spin_until(cooldown.done)
        assert 'cooldown' in cooldown.result().message
        helper.close()
        assert sum(call.args == (29, 1) for call in self.gpio.output.call_args_list) == 1

    def test_peripheral_pin_and_partial_setup_failure(self):
        self.gpio.setup.side_effect = RuntimeError('setup failed')
        helper = self.helper('peripheral')
        self.gpio.setup.assert_called_once_with(31, 1, initial=0)
        self.gpio.output.assert_called_with(31, 0)
        task = self.executor.create_task(helper.reset(Trigger.Request(), Trigger.Response()))
        self.spin_until(task.done)
        assert not task.result().success
        self.gpio.output.assert_called_once_with(31, 0)

    def test_assertion_failure_attempts_release(self):
        helper = self.helper()
        self.gpio.output.side_effect = [RuntimeError('assert failed'), None]
        task = self.executor.create_task(helper.reset(Trigger.Request(), Trigger.Response()))
        self.spin_until(task.done)
        assert not task.result().success
        assert self.gpio.output.call_args_list[-1].args == (29, 0)
        self.gpio.output.side_effect = None

    def test_release_failure_and_cleanup_retry(self):
        helper = self.helper()
        self.gpio.output.side_effect = [None, RuntimeError('release failed'), None]
        task = self.executor.create_task(helper.reset(Trigger.Request(), Trigger.Response()))
        self.spin_until(task.done)
        assert not task.result().success
        helper.close()
        assert self.gpio.output.call_args_list[-1].args == (29, 0)
        self.gpio.output.side_effect = None

    def test_incremental_reads_do_not_wait_for_missing_data(self):
        node = Mock()
        node._serial.in_waiting = 0
        node._serial.read.return_value = b''
        node._line_buffer = bytearray()
        node._line_started = time.monotonic()
        assert SerialNode.readline_nonblocking(node) is None
        node._serial.read.assert_called_once_with(0)
        node._serial.read.return_value = b'V:12'
        assert SerialNode.readline_nonblocking(node) is None
        node._serial.read.return_value = b'\r\n'
        assert SerialNode.readline_nonblocking(node) == 'V:12'

    def test_non_crush_nodes_skip_gpio(self):
        from offboard_comms.peripheral import PeripheralPublisher  # noqa: PLC0415
        from offboard_comms.thrusters import Thrusters  # noqa: PLC0415

        package_root = Path(__file__).resolve().parents[2]

        def resolve(uri: str, **kwargs) -> str:
            del kwargs
            return str(package_root / uri.removeprefix('package://'))

        with patch.dict('os.environ', {'ROBOT_NAME': 'oogway'}), \
                patch('offboard_comms.serial_node.rr.get_filename', side_effect=resolve), \
                patch.object(PeripheralPublisher, 'CONFIG_FILE_PATH', 'package://offboard_comms/config/oogway.yaml'), \
                patch.object(PeripheralPublisher, 'setup_sensors'), \
                patch.object(PeripheralPublisher, 'setup_servos'), \
                patch.object(Thrusters, 'OFFBOARD_COMMS_CONFIG_FILE_PATH',
                             'package://offboard_comms/config/oogway.yaml'), \
                patch.object(Thrusters, 'CONTROLS_CONFIG_FILE_PATH', 'package://controls/config/oogway.yaml'), \
                patch.object(Thrusters, 'load_lookup_tables'):
            for node_class in (PeripheralPublisher, Thrusters):
                node = node_class()
                try:
                    assert node._reset is None
                    self.gpio.setmode.assert_not_called()
                    self.gpio.setup.assert_not_called()
                finally:
                    node.destroy_node()

    def test_serial_failures_retry(self):
        node = Mock()
        node._serial = Mock()
        node._serial.close.side_effect = serial.SerialException('close failed')
        node._line_buffer = bytearray()
        SerialNode.reset_serial(node)
        assert node._serial is None
        node.connect_timer.reset.assert_called_once()
        node._serial = Mock(is_open=True)
        node._serial.write.side_effect = serial.SerialTimeoutException('timeout')
        assert not SerialNode.writebytes(node, b'command')
        node.reset_serial.assert_called_once()
        node.get_ftdi_string.return_value = 'mock'
        with patch('offboard_comms.serial_node.list_ports.grep', return_value=iter([Mock(device='mock')])), \
                patch('offboard_comms.serial_node.serial.Serial', side_effect=serial.SerialException('open failed')):
            SerialNode.connect(node)
        assert node.reset_serial.call_count == 2


if __name__ == '__main__':
    unittest.main()
