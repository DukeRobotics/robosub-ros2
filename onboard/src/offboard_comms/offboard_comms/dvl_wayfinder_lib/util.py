"""Contains several utilities classes.
"""
import math
import threading
import datetime
import os.path
from threading import Thread
from time import sleep
import serial

class Setting:
    """Helper class for data display.
    """
    #pylint: disable=too-many-instance-attributes
    #pylint: disable=too-many-arguments
    INVALID = "INVALID"
    """String for display when data are invalid."""
    NO_DATA = "NO DATA"
    """String for display when there is no new data."""
    def __init__(self, setting_id, name, value, fmt="", units=""):
        self.setting_id = setting_id
        """Setting's ID."""
        self.name = name
        """Name of the setting."""
        self.format = fmt
        """Format string to convert value for display."""
        self.units = units
        """Units for the setting."""
        self.min_value = 0
        """Minimum value used for validation."""
        self.max_value = 0
        """Maximum value used for validation."""
        self.value = None
        """Value of the setting (float, int, str)."""
        self.value_string = ""
        """Display string representing value."""
        self.update(value)

    def set_min_max(self, min_value, max_value):
        """Sets minimum and maximum values.
        """
        self.min_value = min_value
        self.max_value = max_value

    def update(self, value):
        """Updates value and value_string.
        """
        self.value = value
        if isinstance(value, float) and math.isnan(value):
            self.value_string = Setting.INVALID
        elif self.format == "":
            self.value_string = ""
        else:
            self.value_string = self.format.format(value)


class SerialPort():
    """Class responsible for serial communications.

    Parameters
    ----------
    com : str
        String that represents COM port to be opened, for example "COM1".
    baud_rate : int
        Baud-rate to use when opening the port.
    """
    #pylint: disable=too-many-instance-attributes
    _THREAD_DELAY = 0.01

    def __init__(self, com="COM1", baud_rate=115200):
        self.com = com
        """String that represents COM port."""
        self.baudrate = baud_rate
        """Serial port baud rate."""
        self.serial_port = None
        """Serial port object."""
        self._port = None
        self._receive_callback = []
        self._run = False
        self._thread = None
        self._lock = threading.Lock()

    def __enter__(self):
        """Opens port on enter.
        """
        self.close()
        try:
            com = self.com
            if com.startswith("COM"):
                com = "\\\\.\\"+ com
            self._port = serial.Serial(com, self.baudrate)
            self._port.setDTR(True)
            self._port.setRTS(True)
            self._port.writeTimeout = 1

        except serial.SerialException:
            self._port = None

        if (self._port is not None) and self._port.isOpen() and not self._run:
            self._run = True
            self._thread = Thread(target=self._receive_listener, name="Wayfinder serial thread")
            self._thread.daemon = True
            self._thread.start()
        else:
            self._port = None

    def __exit__(self, exc_type, exc_value, traceback):
        """Closes port on exit.
        """
        self.close()

    def open(self, com: str, baud_rate: int) -> bool:
        """Opens serial port.

        Parameters
        ----------
        com : str
            String that represents COM port to be opened, for example "COM1".
        baud_rate : int
            Baud-rate to use when opening the port.

        Returns
        -------
        bool
            True if port is opened, False otherwise.
        """
        self.com = com
        self.baudrate = baud_rate
        self.__enter__()
        return self._run

    def is_open(self) -> bool:
        """Checks if port is open.

        Returns
        -------
        bool
            True if port is opened, False otherwise.
        """
        return self._run

    def close(self):
        """Closes serial port.
        """
        if self._run:
            self._run = False
            self._thread.join(2)
        if self._port is not None and self._port.isOpen():
            self._port.close()
            self.serial_port = None

    def write(self, array: bytearray):
        """Writes byte array to port.

        Parameters
        ----------
        array : byte array
            The byte array to be written to serial port.
        """
        if self._port is not None and self._port.isOpen():
            with self._lock:
                try:
                    self._port.write(array)
                except serial.SerialException:
                    pass

    def register_receive_callback(self, function):
        """Registers receive callback function.

        Parameters
        ----------
        function
            Callback function.
        """
        self._receive_callback.append(function)

    def unregister_all_callbacks(self):
        """Unregisters all callback functions.
        """
        self._receive_callback.clear()

    def _receive_listener(self):
        """Thread function that reads bytes from port.
        """
        while self._run:
            if self._port.isOpen():
                bytes_to_read = self._port.inWaiting()
                if bytes_to_read > 0:
                    with self._lock:
                        arr = self._port.read_all()
                    if self._run and len(arr) > 0:
                        for func in self._receive_callback:
                            func(arr)
            sleep(SerialPort._THREAD_DELAY)
        #print("Finished serial thread")

    def set_baudrate(self, baud_rate: int) -> bool:
        """Changes baud-rate of the open port.

        Parameters
        ----------
        baud_rate : int
            Changes the baud rate and re-opens the port.

        Returns
        -------
        bool
            True if port is opened, False otherwise.
        """
        self.baudrate = baud_rate
        if self._port is not None:
            self.close()
            sleep(0.1)
        return self.open(self.com, self.baudrate)

class DataLogger():
    """ Class responsible for data logging.
    """
    def __init__(self):
        self._log_file = None
        self._log_file_name = ""

    def is_logging(self) -> bool:
        """Checks if data are logged.

        Returns
        -------
        bool
            True if file is opened for logging, False otherwise.
        """
        return self._log_file is not None

    def open_file(self, working_folder: str, prefix: str, ext=".pd") -> str:
        """Creates file for data logging - name is auto generated (includes time stamp).

        Parameters
        ----------
        working_folder : str
            A name of the folder where file will be opened.
        prefix : str
            A string to appended to front of the file name.
        ext : str
            Extension of the log file name.

        Returns
        -------
        str
            On successful creation of the file returns file path.
            If the operation fails it returns None.
        """
        now = datetime.datetime.now()
        name = prefix + now.strftime("%Y-%m-%d_%H%M%S") + ext
        name = os.path.join(working_folder, name)
        self._log_file = open(name, "ab+")

        if not os.path.exists(name):
            self._log_file = None
            self._log_file_name = ""
            return None
        self._log_file_name = name
        return name

    def open_file_with_name(self, name: str, append=True) -> str:
        """Creates file for logging with user specified name.

        Parameters
        ----------
        name : str
            A name (including path) of the file where data will be logged.

        Returns
        -------
        str
            On successful creation of the file returns file path.
            If the operation fails it returns None.
        """
        if append:
            self._log_file = open(name, "ab+")
        else:
            self._log_file = open(name, "wb")

        if not os.path.exists(name):
            self._log_file = None
            self._log_file_name = ""
            return None
        self._log_file_name = name
        return name

    def close_file(self):
        """Closes log file.
        """
        if self._log_file is not None:
            self._log_file.close()
            self._log_file = None
            self._log_file_name = ""

    def write(self, array: bytearray):
        """Writes byte array to log data file.

        Parameters
        ----------
        array : byte array
            The byte array to be written to log file.
        """
        if self._log_file is not None and not self._log_file.closed:
            try:
                self._log_file.write(array)
                self._log_file.flush()
            except ValueError:
                pass

    def write_text(self, text: str):
        """Writes text to log data file.

        Parameters
        ----------
        text : str
            The string to be written to log file.
        """
        if self._log_file is not None and not self._log_file.closed:
            try:
                self._log_file.write(text.encode("utf-8"))
                self._log_file.flush()
            except ValueError:
                pass
def print_bytes(array: bytearray) -> str:
    """Outputs byte array and formats for pretty print.

    16 bytes are printed per line with additional space between 8 byte groups
    so that they are easier to distinguish.
    Example:
    AA 10 01 1B   00 02 03 14
    00 02 00 00   1F 23 10 0C
    00 00 00 14   03 1B 09 14
    2E CC 01

    Parameters
    ----------
    array : byte array
        The byte array to be pretty printed.

    Returns
    -------
    str
        A pretty string representation of the byte array.
    """
    strings = ["{:02X}".format(byte) for byte in array]
    output = ""
    for index, byte_string in enumerate(strings):
        if index % 8 == 0 and index != 0:
            output += "\n"
        elif index % 8 == 4:
            output += "  "
        output += byte_string + " "
    return output

def print_bytearray(array: bytearray) -> str:
    """Outputs string of all bytes in HEX without any formatting.

    Parameters
    ----------
    array : byte array
        The byte array to be printed.

    Returns
    -------
    str
        A string representation of the byte array in HEX format.
    """
    strings = ["{:02X}".format(byte) for byte in array]
    output = "".join(strings)
    return output

## Indents string for printing
def indent_string(string: str) -> str:
    """Indents string for printing.

    Parameters
    ----------
    string : str
        Input string to be printed.

    Returns
    -------
    str
        A string representation of lines with indentation.
    """
    output = ""
    for line in string.splitlines():
        output += ("    {0}\n".format(line))
    return output
