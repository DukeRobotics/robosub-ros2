"""Contains several classes related to sending commands to Wayfinder.
"""
from enum import Enum
import datetime
import struct
from queue import Queue, Empty
from dvl.packets import PhysicalLayerPacket, AppLayerPacket, PacketDecoder, AppLayerIdType
from dvl.util import DataLogger, SerialPort
from dvl.system import SystemInfo, SystemFeatures, SystemSetup, SystemTests,\
   SystemComponents, SystemUpdate, DateTime, FftTest, OutputData, FftData

class CommandIdType(Enum):
    """Enumerated type class that defines command IDs.
    """
    ENTER_CMD_MODE = 0x0000000F
    """Enter command mode (stop pinging) command ID."""
    EXIT_CMD_MODE = 0x00000010
    """Exit command mode (start pinging) command ID."""
    SOFTWARE_TRIGGER = 0x00000011
    """Software trigger command ID."""
    GET_TIME = 0x1D000001
    """Get system time command ID."""
    SET_TIME = 0x1F000002
    """Set system time command ID."""
    GET_SYSTEM = 0x81000001
    """Get system information command ID."""
    SET_SYSTEM = 0x83000002
    """Set system information command ID."""
    GET_SETUP = 0x85000001
    """Get system setup command ID."""
    SET_SETUP = 0x87000002
    """Set system setup command ID."""
    GET_TESTS = 0x19000001
    """Get test results command ID."""
    GET_FEATURES = 0x10000001
    """Get features command ID."""
    SET_FEATURES = 0x12000002
    """Set features command ID."""
    GET_FFT = 0x1B000002
    """Get FFT test command ID."""
    RESET_TO_DEFAULTS = 0x00000017
    """Reset to factory defaults command ID."""
    SET_SPEED_OF_SOUND = 0x86000003
    """Set speed of sound command ID."""
    FIRMWARE_UPDATE = 0x22000001
    """Start firmware update command ID."""
    UPLOAD_FILE = 0x22000002
    """Upload firmware file command ID."""
    GET_COMPONENTS = 0x81000003
    """Get hardware components information command ID."""
    SET_COMPONENTS = 0x83000004
    """Set hardware components information (internal only) command ID."""

class ResponseStatusType(Enum):
    """Enumerated type class that defines command response status.
    """
    NO_RESPONSE = 0
    """No response from the system."""
    SUCCESS = 1
    """Successful execution."""
    UNKNOWN_CMD = 2
    """Unknown command."""
    PARAM_INVALID = 3
    """One or more parameters are invalid."""
    CMD_EXEC_ERR = 4
    """Error in command execution."""
    CMD_SET_ERR = 5
    """Set command error."""
    CMD_GET_ERR = 6
    """Get command error."""
    NORUN_WITH_PING = 7
    """Cannot execute command while pinging."""
    CANNOT_OPEN_PORT = 100
    """Cannot open COM port."""

class ResponseErrorType(Enum):
    """Enumerated type class that defines command response error.
    """
    INVALID_NONE = 0
    """No error - All parameters are valid."""
    INVALID_BAUD = 1
    """Invalid baud rate."""
    INVALID_TRIGGER = 2
    """Invalid trigger enable/disable value."""
    INVALID_SOS = 3
    """Invalid speed of sound."""
    INVALID_MAXDEPTH = 4
    """Invalid maximum depth."""
    INVALID_DATETIME = 5
    """Invalid date/time."""

class Communicator():
    """Class that implements binary communication layer.
    """
    #pylint: disable=too-many-instance-attributes

    _MAX_Q_SIZE = 1000

    def __init__(self, port=None):
        if port is None:
            self.port = SerialPort()
        else:
            self.port = port
        self.data_logger = DataLogger()
        self.all_data_logger = DataLogger()
        self._decoder = PacketDecoder()
        self._fft_queue = Queue(Communicator._MAX_Q_SIZE)
        self._cmd_queue = Queue(Communicator._MAX_Q_SIZE)
        self._status_queue = Queue(Communicator._MAX_Q_SIZE)
        self.port.register_receive_callback(self.decode_packets)
        self._ondata_callback = []
        self._cmd_count = 0

    def __enter__(self):
        """Initializes serial port interface.
        """
        self.port.__enter__()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        """Cleans up serial port interface.
        """
        self.port.__exit__(exc_type, exc_value, traceback)

    def decode_packets(self, arr):
        """Decodes binary packets and puts them into queues.
        """
        pl_packets = self._decoder.parse_bytes(arr)
        if self.all_data_logger.is_logging():
            self.all_data_logger.write(arr)
        for pkt in pl_packets:
            al_pkt = pkt.payload
            if al_pkt.pkt_id in (AppLayerIdType.CMD_BIN, AppLayerIdType.RSP_BIN):
                #print(pkt)
                if self._cmd_queue.qsize() >= Communicator._MAX_Q_SIZE:
                    self._cmd_queue.get_nowait()
                self._cmd_queue.put(al_pkt, block=False)
                self._cmd_count += 1
                #print("Got response {0}".format(self._cmd_count))
            elif al_pkt.pkt_id == AppLayerIdType.DATA_PD:
                payload = al_pkt.get_payload()
                output_data = OutputData(payload)
                if output_data.is_valid:
                    self._call_back(output_data)
                    if self.data_logger.is_logging():
                        self.data_logger.write(pkt.encode())
            elif al_pkt.pkt_id == AppLayerIdType.FFT_DATA:
                if self._fft_queue.qsize() >= Communicator._MAX_Q_SIZE:
                    self._fft_queue.get_nowait()
                fft_data = FftData(al_pkt.get_payload())
                self._fft_queue.put(fft_data, block=False)
            elif al_pkt.pkt_id == AppLayerIdType.STATUS:
                if self._status_queue.qsize() >= Communicator._MAX_Q_SIZE:
                    self._status_queue.get_nowait()
                self._status_queue.put(al_pkt, block=False)

    #pylint: disable=broad-except
    def _call_back(self, output_data):
        try:
            for callback in self._ondata_callback:
                func = callback[0]
                obj = callback[1]
                func(output_data, obj)
        except Exception as exception:
            print("Exception in callback function: {0}".format(exception))

    def send_packet(self, packet: PhysicalLayerPacket):
        """Sends physical layer packet to DVL.
        """
        self.port.write(packet.encode())

    def flush_fft_queue(self):
        """Flushes FFT data queue.
        """
        while not self._fft_queue.empty():
            self._fft_queue.get_nowait()

    def flush_cmd_queue(self):
        """Flushes command queue.
        """
        while not self._cmd_queue.empty():
            self._cmd_queue.get_nowait()

    def flush_status_queue(self):
        """Flushes status queue.
        """
        while not self._status_queue.empty():
            self._status_queue.get_nowait()

    def get_cmd_packet(self, time_out: int = 0) -> AppLayerPacket:
        """Gets command response packet.
        """
        if time_out == 0 and self._cmd_queue.empty():
            return None
        try:
            return self._cmd_queue.get(block=True, timeout=time_out)
        except Empty:
            self._decoder.clear()
            return None

    def get_fft_count(self):
        """Returns number of available FFT packets.
        """
        return self._fft_queue.qsize()

    def get_fft_packet(self, time_out):
        """Gets the FFT packet from queue.
        """
        try:
            return self._fft_queue.get(block=True, timeout=time_out)
        except Empty:
            return None

    def get_status_count(self):
        """Returns number of available status packets.
        """
        return self._status_queue.qsize()

    def get_status_packet(self, time_out):
        """Gets the data packet from queue.
        """
        try:
            return self._status_queue.get(block=True, timeout=time_out)
        except Empty:
            return None

    def send_and_wait_for_response(self, packet: AppLayerPacket, \
            timeout: int = 0, debug: bool = False) -> AppLayerPacket:
        """Sends packet and waits for response.
        """
        pl_pkt = PhysicalLayerPacket(packet)
        if debug:
            print(pl_pkt)
        if self.all_data_logger.is_logging():
            self.all_data_logger.write(bytearray(b"<Sent cmd>"))
        self.flush_cmd_queue()
        self.send_packet(pl_pkt)
        if timeout >= 0:
            response = self.get_cmd_packet(timeout)
            return response
        return None

    def register_ondata_callback(self, func, obj):
        """Registers receive callback function.
        """
        self._ondata_callback.append((func, obj))

    def unregister_all_callbacks(self):
        """Unregisters all callback functions.
        """
        self._ondata_callback.clear()

    def reset(self):
        """Resets queues and decoder.
        """
        self.flush_cmd_queue()
        self.flush_status_queue()
        self.flush_fft_queue()
        self._decoder.clear()

COMMAND_TIMEOUT_SEC = 5
"""Timeout for most commands (5 sec)"""

LONG_COMMAND_TIMEOUT = 15
"""Timeout for commands taking longer (15 sec)"""

class BinaryCommands(Communicator):
    """Binary commands interface.
    """
    def __init__(self, port=None):
        Communicator.__init__(self, port)

    def enter_command_mode(self) -> ResponseStatusType:
        """Enters command mode (stops pinging).
        """
        (err, _) = self.send_cmd(CommandIdType.ENTER_CMD_MODE, LONG_COMMAND_TIMEOUT)
        return err

    def exit_command_mode(self) -> ResponseStatusType:
        """Exits command  mode (start pinging).
        """
        (err, _) = self.send_cmd(CommandIdType.EXIT_CMD_MODE)
        return err

    def reset_to_defaults(self) -> ResponseStatusType:
        """Resets to factory defaults.
        """
        (err, _) = self.send_cmd(CommandIdType.RESET_TO_DEFAULTS)
        return err

    def get_time(self):
        """Gets system time.
        """
        (err, response) = self.send_cmd(CommandIdType.GET_TIME)
        if err.value != ResponseStatusType.SUCCESS.value:
            return (err, None)
        date_time = DateTime.decode(response)
        return (err, date_time)

    def set_time(self, date_time: datetime):
        """Sets system time.
        """
        cmd_id = CommandIdType.SET_TIME
        al_pkt = DateTime.encode(date_time, cmd_id.value)
        response = self.send_and_wait_for_response(al_pkt, COMMAND_TIMEOUT_SEC)
        return check_response(response, cmd_id)

    def set_speed_of_sound(self, value: float):
        """Sets speed of sound.
        """
        arr = bytearray(8)
        struct.pack_into("I", arr, 0, CommandIdType.SET_SPEED_OF_SOUND.value)
        struct.pack_into("f", arr, 4, value)
        al_pkt = AppLayerPacket()
        al_pkt.create_from_payload(AppLayerIdType.CMD_BIN, arr)
        response = self.send_and_wait_for_response(al_pkt, COMMAND_TIMEOUT_SEC)
        return check_response(response, CommandIdType.SET_SPEED_OF_SOUND)

    def get_system(self):
        """Gets DVL system information.
        """
        (err, response) = self.send_cmd(CommandIdType.GET_SYSTEM)
        if err.value != ResponseStatusType.SUCCESS.value:
            return (err, None)
        info = SystemInfo()
        info.decode(response)
        return (err, info)

    def get_components(self):
        """Gets DVL hardware components information.
        """
        (err, response) = self.send_cmd(CommandIdType.GET_COMPONENTS)
        if err.value != ResponseStatusType.SUCCESS.value:
            return (err, None)
        info = SystemComponents()
        info.decode(response)
        return (err, info)

    def get_tests(self):
        """Gets DVL system tests.
        """
        (err, response) = self.send_cmd(CommandIdType.GET_TESTS, LONG_COMMAND_TIMEOUT)
        if err.value != ResponseStatusType.SUCCESS.value:
            return (err, None)
        tests = SystemTests()
        tests.decode(response)
        return (err, tests)

    def get_setup(self):
        """Gets user setup.
        """
        (err, response) = self.send_cmd(CommandIdType.GET_SETUP)
        if err.value != ResponseStatusType.SUCCESS.value:
            return (err, None)
        info = SystemSetup()
        info.decode(response)
        return (err, info)

    def set_setup(self, setup: SystemSetup):
        """Sets user setup.
        """
        cmd_id = CommandIdType.SET_SETUP
        al_pkt = SystemSetup.encode(setup, cmd_id.value)
        response = self.send_and_wait_for_response(al_pkt, COMMAND_TIMEOUT_SEC)
        return check_response(response, cmd_id)

    def get_features(self):
        """Gets DVL system features.
        """
        (err, response) = self.send_cmd(CommandIdType.GET_FEATURES)
        if err.value != ResponseStatusType.SUCCESS.value:
            return (err, None)
        features = SystemFeatures()
        features.decode(response)
        return (err, features)

    def set_system_features(self, feature_code: bytearray):
        """Sets DVL features.
        """
        cmd_id = CommandIdType.SET_FEATURES
        al_pkt = SystemFeatures.encode(feature_code, cmd_id.value)
        response = self.send_and_wait_for_response(al_pkt, COMMAND_TIMEOUT_SEC)
        return check_response(response, cmd_id)

    def send_software_trigger(self):
        """Sends software trigger to ping
        """
        (err, _) = self.send_cmd(CommandIdType.SOFTWARE_TRIGGER)
        return err

    def send_cmd(self, cmd: CommandIdType, \
            timeout=COMMAND_TIMEOUT_SEC) \
            -> (ResponseStatusType, AppLayerPacket):
        """Sends command and waits for response.
        """
        al_pkt = _create_cmd(cmd)
        response = self.send_and_wait_for_response(al_pkt, timeout)
        err = check_response(response, cmd)
        return (err, response)

    def get_fft_test(self):
        """Gets FFT samples.
        """
        setup = FftTest()
        al_pkt = FftTest.encode(setup, CommandIdType.GET_FFT.value)
        mult = 1
        if self.port.baudrate == 9600:
            mult = 4
        self.reset()
        response = self.send_and_wait_for_response(al_pkt, LONG_COMMAND_TIMEOUT * mult)
        err = check_response(response, CommandIdType.GET_FFT)

        if err.value != ResponseStatusType.SUCCESS.value:
            return (err, None)
        fft = self.get_fft_packet(COMMAND_TIMEOUT_SEC)
        return (err, fft)

    def start_system_update(self, file_size: int, chunk_size: int):
        """Starts firmware update.
        """
        cmd_id = CommandIdType.FIRMWARE_UPDATE
        al_pkt = SystemUpdate.encode(file_size, chunk_size, cmd_id.value)
        response = self.send_and_wait_for_response(al_pkt, COMMAND_TIMEOUT_SEC)
        return check_response(response, cmd_id)

    def upload_file(self, arr: bytearray, chunk_size: int):
        """Uploads chunk of the file.
        """
        cmd_id = CommandIdType.UPLOAD_FILE
        al_pkt = SystemUpdate.encode_data(arr, cmd_id.value, chunk_size)
        response = self.send_and_wait_for_response(al_pkt, \
            LONG_COMMAND_TIMEOUT)
        return check_response(response, cmd_id)

    def send_cmd_without_wait(self, cmd: CommandIdType) \
            -> (ResponseStatusType, AppLayerPacket):
        """Sends command without waiting for response.
        """
        al_pkt = _create_cmd(cmd)
        self.send_and_wait_for_response(al_pkt, -1)

def _create_cmd(cmd: CommandIdType) -> AppLayerPacket:
    """Creates command as application layer packet.
    """
    payload = bytearray(4)
    struct.pack_into("I", payload, 0, cmd.value)
    al_pkt = AppLayerPacket()
    al_pkt.create_from_payload(AppLayerIdType.CMD_BIN, payload)
    return al_pkt

def check_response(packet: AppLayerPacket, expected_cmd: CommandIdType) -> ResponseStatusType:
    """Checks command response.
    """
    if packet is None:
        return ResponseStatusType.NO_RESPONSE
    if len(packet.payload) < 5:
        return ResponseStatusType.UNKNOWN_CMD
    [cmd_id] = struct.unpack("I", packet.payload[0:4])
    if cmd_id != expected_cmd.value:
        return ResponseStatusType.UNKNOWN_CMD
    try:
        err = ResponseStatusType(packet.payload[4])
    except ValueError:
        err = ResponseStatusType.UNKNOWN_CMD
    return err
