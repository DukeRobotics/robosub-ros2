import time

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.task import Future
from std_srvs.srv import Trigger


class ArduinoReset:
    """Own one Arduino reset output and its asynchronous service on an existing node."""

    MAX_COOLDOWN = 60

    def __init__(self, node: Node, board: str, config: dict) -> None:
        self._node = node
        self._board = board
        self._gpio = None
        self._pin = None
        self._ready = False
        self._pending = None
        self._timer = None
        self._last_reset = float('-inf')
        self._group = ReentrantCallbackGroup()
        self._service = node.create_service(
            Trigger, f'/offboard/reset_{board}', self.reset, callback_group=self._group,
        )
        try:
            self._configure(config)
        except Exception as exc:  # noqa: BLE001 Keep GPIO failures isolated from serial communication.
            node.get_logger().error(f'{board} reset unavailable: {exc}')
            self._release()

    def _configure(self, config: dict) -> None:
        """Initialize only this board's output, released LOW."""
        pins = config['pins']
        self._pulse = float(config['pulse_seconds'])
        self._cooldown = float(config['cooldown_seconds'])
        if not 0 < self._pulse <= 1 or not self._pulse <= self._cooldown <= self.MAX_COOLDOWN:
            error_msg = 'Reset timing must satisfy 0 < pulse <= 1 and pulse <= cooldown <= 60 seconds.'
            raise ValueError(error_msg)
        if set(pins) != {'thruster', 'peripheral'}:
            error_msg = 'Reset pins must specify thruster and peripheral.'
            raise ValueError(error_msg)
        # Dedicated GPIO header pins verified against the Orin Nano pin table.
        if any(type(pin) is not int or pin not in (29, 31) for pin in pins.values()):
            error_msg = 'Reset pins must use physical header pins 29 and 31.'
            raise ValueError(error_msg)
        if len(set(pins.values())) != len(pins):
            error_msg = 'Reset pins must be distinct.'
            raise ValueError(error_msg)

        self._pin = pins[self._board]
        from Jetson import GPIO  # noqa: PLC0415 Import hardware support only on Crush.

        self._gpio = GPIO
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self._pin, GPIO.OUT, initial=GPIO.LOW)
        self._ready = True

    def _release(self) -> bool:
        """Attempt release even after partial initialization or assertion failure."""
        if self._gpio is None or self._pin is None:
            return False
        try:
            self._gpio.output(self._pin, self._gpio.LOW)
        except Exception as exc:  # noqa: BLE001 Report hardware errors without stopping serial communication.
            self._node.get_logger().error(f'Could not release reset pin {self._pin}: {exc}')
            self._ready = False
            return False
        return True

    async def reset(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """Assert on explicit request and yield until a steady-clock timer releases reset."""
        del request
        response.success = False
        if not self._ready:
            response.message = f'{self._board} GPIO reset unavailable; check GPIO setup and restart this node.'
        elif self._pending is not None:
            response.message = f'{self._board} reset already in progress.'
        elif time.monotonic() - self._last_reset < self._cooldown:
            response.message = f'{self._board} reset cooldown active.'
        else:
            self._last_reset = time.monotonic()
            pending = Future()
            self._pending = pending
            try:
                self._gpio.output(self._pin, self._gpio.HIGH)
                self._timer = self._node.create_timer(
                    self._pulse, self._finish, clock=Clock(clock_type=ClockType.STEADY_TIME),
                )
            except Exception as exc:  # noqa: BLE001 Always attempt release after pulse errors.
                self._release()
                self._pending = None
                response.message = f'{self._board} GPIO reset failed: {exc}'
            else:
                response.success = await pending
                response.message = (
                    f'{self._board} reset pulse completed; Arduino readiness is not verified.'
                    if response.success else f'{self._board} GPIO reset release failed or node stopped.'
                )
        return response

    def _finish(self) -> None:
        """Release the output before completing the waiting service response."""
        released = self._release()
        if self._timer is not None:
            self._node.destroy_timer(self._timer)
            self._timer = None
        if self._pending is not None:
            self._pending.set_result(released and self._ready)
            self._pending = None

    def close(self) -> None:
        """Release and clean up this board's pin without ever asserting reset."""
        self._ready = False
        self._finish()
        if self._gpio is not None and self._pin is not None:
            try:
                self._gpio.cleanup(self._pin)
            except Exception as exc:  # noqa: BLE001 Continue node shutdown after hardware errors.
                self._node.get_logger().error(f'Could not clean up reset pin {self._pin}: {exc}')
