"""Contains several classes that hold information from Wayfinder.
"""
#pylint: disable=too-many-lines
#pylint: disable=too-many-instance-attributes
#pylint: disable=too-many-statements
import math
import struct
import datetime
from enum import Enum, auto
import numpy as np
from dvl.packets import AppLayerPacket, AppLayerIdType, calc_checksum
from dvl.util import Setting, indent_string

class DateTime():
    """Date and time helper class for getting/setting system time.
    """
    _STRUCTURE_ID = 0x23
    _VERSION = 0x10
    _SIZE = 12
    _OFFSET = 6

    @classmethod
    def decode(cls, packet: AppLayerPacket):
        """Decodes date and time from application layer packet.
        """
        arr = packet.get_payload()
        if len(arr) < DateTime._SIZE:
            return None
        arr = arr[DateTime._OFFSET:]
        try:
            [size] = struct.unpack("I", arr[2:6])
            if size != DateTime._SIZE:
                return None
            year = arr[6]
            if year < 99:
                year += 2000
            month = arr[7]
            day = arr[8]
            hour = arr[9]
            minutes = arr[10]
            second = arr[11]
            if day <= 0 or day > 31 or month <= 0 or month > 12:
                date_time = None
            else:
                date_time = datetime.datetime(year, month, day, hour, minutes, second, 0)
        except ValueError:
            date_time = None
        return date_time

    @classmethod
    def encode(cls, date_time: datetime, cmd_id: int) -> AppLayerPacket:
        """Encodes date and time into application layer packet.

        Parameters
        ----------
        date_time : datetime
            Date and time to set system clock.
        cmd_id : int
            Command ID for setting the system time.

        Returns
        -------
        AppLayerPacket
            Application layer packet class.
        """
        arr = bytearray(DateTime._SIZE+4)
        struct.pack_into("I", arr, 0, cmd_id)
        year = date_time.year
        year = year - int(year/100) * 100
        arr[4] = DateTime._STRUCTURE_ID
        arr[5] = DateTime._VERSION
        struct.pack_into("I", arr, 6, DateTime._SIZE)
        arr[10] = year
        arr[11] = date_time.month
        arr[12] = date_time.day
        arr[13] = date_time.hour
        arr[14] = date_time.minute
        arr[15] = date_time.second
        pkt = AppLayerPacket()
        pkt.create_from_payload(AppLayerIdType.CMD_BIN, arr)
        return pkt

class SystemInfo:
    """Class that contains system information.
    """
    #pylint: disable=too-many-instance-attributes

    _STRUCTURE_ID = 0x21
    _VERSION = 0x10
    _SIZE = 135
    _OFFSET = 6

    def __init__(self):
        self.is_valid = False
        """Defines if data in the class are valid (i.e. they are decoded properly)"""
        self.struct_id = SystemInfo._STRUCTURE_ID
        """Structure ID number."""
        self.version = SystemInfo._VERSION
        """Structure version number."""
        self.size = SystemInfo._SIZE
        """Structure size."""
        self.frequency = 0
        """System frequency in Hz."""
        self.fw_major_version = 0
        """Firmware major number."""
        self.fw_minor_version = 0
        """ Firmware minor number."""
        self.fw_patch_version = 0
        """Firmware patch number."""
        self.fw_build_version = 0
        """Firmware build number."""
        self.fpga_version = 0
        """FPGA firmware number."""
        self.system_id = 0
        """System ID number."""
        self.xducer_type = 0
        """Transducer type."""
        self.beam_angle = 30
        """Beam angle in degrees (for Wayfinder 30 deg)."""
        self.has_vertical_beam = False
        """Defines if vertical beam is present."""
        self.system_type = 76
        """System type (for Wayfinder 76)."""
        self.system_subtype = 0
        """System sub-type (for Wayfinder 0)."""

    def __str__(self):
        return (
            "System Info\n"
            "-----------\n"
            "Structure ID     : 0x{0:02X}       ({0})\n"
            "Version          : 0x{1:02X}       ({1})\n"
            "Size             : 0x{2:08X} ({2})\n"
            "Frequency        : {3} Hz\n"
            "Firmware Version : {4}.{5}.{6}.{7}\n"
            "FPGA Version     : 0x{8:08X}\n"
            "System ID        : 0x{9:016X}\n"
            "Transducer Type  : {10}\n"
            "Beam Angle       : {11} degrees\n"
            "Vertical Beam    : {12}\n"
        ).format(
            self.struct_id,
            self.version,
            self.size,
            self.frequency,
            self.fw_major_version,
            self.fw_minor_version,
            self.fw_patch_version,
            self.fw_build_version,
            self.fpga_version,
            self.system_id,
            self.xducer_type,
            self.beam_angle,
            self.has_vertical_beam,
        )

    def to_string(self):
        """Outputs information to string for logging
        """
        return (
            "System Info\n"
            "-----------\n"
            "Frequency        : {} Hz\n"
            "Firmware Version : {}\n"
            "FPGA Version     : 0x{:08X}\n"
            "System ID        : 0x{:016X}\n"
            "Transducer Type  : {}\n"
            "Beam Angle       : {} degrees\n"
            "Vertical Beam    : {}\n"
        ).format(
            self.frequency,
            self.get_fw_version(),
            self.fpga_version,
            self.system_id,
            self.xducer_type,
            self.beam_angle,
            self.has_vertical_beam
        )
    def decode(self, packet: AppLayerPacket):
        """Decodes system info from application layer packet.
        """
        arr = packet.get_payload()
        self.decode_from_array(arr)

    def decode_from_array(self, arr: bytearray):
        """Decodes system info from byte array.
        """
        length = len(arr)
        if length < SystemInfo._SIZE + SystemInfo._OFFSET - 2:
            return
        arr = arr[SystemInfo._OFFSET:]
        try:
            self.struct_id = arr[0]
            self.version = arr[1]
            [self.size] = struct.unpack("I", arr[2:6])
            [self.frequency] = struct.unpack("f", arr[6:10])
            self.fw_major_version = arr[10]
            self.fw_minor_version = arr[11]
            self.fw_patch_version = arr[12]
            self.fw_build_version = arr[13]
            [self.fpga_version] = struct.unpack("I", arr[14:18])
            [self.system_id] = struct.unpack("Q", arr[18:26])
            self.xducer_type = arr[26]
            [self.beam_angle] = struct.unpack("f", arr[27:31])
            self.has_vertical_beam = False
            if arr[31] > 0:
                self.has_vertical_beam = True

            if self.size == SystemInfo._SIZE:
                self.system_type = arr[133]
                self.system_subtype = arr[134]
            self.is_valid = True

        except ValueError:
            self.is_valid = False

    def get_fw_version(self):
        """Returns firmware version string in format 'x.xx.xx.xx'.
        """
        if self.is_valid:
            return "{0:d}.{1:02d}.{2:02d}.{3:02d}".format(
                self.fw_major_version, self.fw_minor_version,
                self.fw_patch_version, self.fw_build_version)
        return "-"

    def get_settings(self):
        """Returns system info in form of settings.
        """
        settings = []

        setting = Setting(0, "Frequency", self.frequency * 0.001, "{0:.0f}", "kHz")
        settings.append(setting)

        setting = Setting(1, "Firmware version", self.get_fw_version(), "{0:s}")
        settings.append(setting)

        setting = Setting(2, "FPGA version", self.fpga_version, "0x{0:X}")
        settings.append(setting)

        setting = Setting(3, "System ID", self.system_id, "0x{:016X}")
        settings.append(setting)

        if not self.is_valid:
            for setting in settings:
                setting.value_string = "-"

        return settings

class SystemComponents:
    """Class that contains system hardware components information.
    """
    #pylint: disable=too-many-instance-attributes

    _STRUCTURE_ID = 0x29
    _VERSION = 0x10
    _SIZE = 133
    _OFFSET = 6
    _NUM_HARDWARE = 4

    def __init__(self):
        self.is_valid = False
        """Defines if data in the class are valid (i.e. they are decoded properly)"""
        self.struct_id = SystemComponents._STRUCTURE_ID
        """Structure ID number."""
        self.version = SystemComponents._VERSION
        """Structure version number."""
        self.size = SystemComponents._SIZE
        """Structure size."""
        self.num_hardware = 0
        """ Number of hardware boards."""
        self.hardware_pn = []
        """List of part numbers for hardware."""
        self.hardware_rev = []
        """List of revision numbers for hardware."""
        self.hardware_sn = []
        """List of serial numbers for hardware."""
        for _ in range(self._NUM_HARDWARE):
            self.hardware_pn.append("")
            self.hardware_rev.append("")
            self.hardware_sn.append("")

    def __str__(self):
        boards = ""
        for i in range(self._NUM_HARDWARE):
            boards += "{0}{1} : {2}\n".format(self.hardware_pn[i],\
                self.hardware_rev[i], self.hardware_sn[i])
        return (
            "System Components\n"
            "-----------\n"
            "Structure ID     : 0x{0:02X}       ({0})\n"
            "Version          : 0x{1:02X}       ({1})\n"
            "Size             : 0x{2:08X} ({2})\n"
            "Components       : {3}\n"
            "Boards           :\n"
            "{4}"
        ).format(
            self.struct_id,
            self.version,
            self.size,
            self.num_hardware,
            indent_string(boards)
        )

    def to_string(self):
        """Outputs information to string for logging
        """
        boards = ""
        for i in range(self._NUM_HARDWARE):
            boards += "{0}{1} : {2}\n".format(self.hardware_pn[i],\
                self.hardware_rev[i], self.hardware_sn[i])
        return (
            "System Components\n"
            "-----------\n"
            "Components       : {}\n"
            "Boards           :\n"
            "{}\n"
        ).format(
            self.num_hardware,
            indent_string(boards)
        )

    def decode(self, packet: AppLayerPacket):
        """Decodes system info from application layer packet.
        """
        arr = packet.get_payload()
        self.decode_from_array(arr)

    def decode_from_array(self, arr: bytearray):
        """Decodes system info from byte array.
        """
        length = len(arr)
        if length < SystemComponents._SIZE + SystemComponents._OFFSET - 2:
            return
        arr = arr[SystemComponents._OFFSET:]
        try:
            self.struct_id = arr[0]
            self.version = arr[1]
            [self.size] = struct.unpack("I", arr[2:6])
            self.num_hardware = arr[6]
            offset = 7
            self.hardware_pn = []
            self.hardware_rev = []
            self.hardware_sn = []
            if self.num_hardware == SystemComponents._NUM_HARDWARE:
                for _ in range(SystemComponents._NUM_HARDWARE):
                    limit = offset+32
                    data = arr[offset:limit]
                    data_bin = data.strip(b'\x00').split(b"\x00")
                    if len(data_bin) >= 3:
                        self.hardware_pn.append(data_bin[0].decode("utf-8", errors='ignore'))
                        self.hardware_rev.append(data_bin[1].decode("utf-8", errors='ignore'))
                        self.hardware_sn.append(data_bin[2].decode("utf-8", errors='ignore'))
                    offset = limit

            self.is_valid = True

        except ValueError:
            self.is_valid = False

    def get_index(self, string):
        """Returns index of string in part number.
        """
        for i in range(len(self.hardware_pn)):
            if string in self.hardware_pn[i]:
                return i
        return 0

    def get_settings(self):
        """Returns system info in form of settings.
        """
        settings = []

        cpu = self.get_index("CPU")
        xdr = self.get_index("XDR")
        dsc = self.get_index("DSC")
        sys = self.get_index("SYS")

        setting = Setting(0, "CPU part number", self.hardware_pn[cpu][3:], "{0:11s}")
        settings.append(setting)

        setting = Setting(1, "XDR part number", self.hardware_pn[xdr][3:], "{0:11s}")
        settings.append(setting)

        setting = Setting(2, "DSC part number", self.hardware_pn[dsc][3:], "{0:11s}")
        settings.append(setting)

        setting = Setting(3, "SYS part number", self.hardware_pn[sys][3:], "{0:11s}")
        settings.append(setting)

        setting = Setting(4, "CPU revision", self.hardware_rev[cpu], "{0:4s}")
        settings.append(setting)

        setting = Setting(5, "XDR revision", self.hardware_rev[xdr], "{0:4s}")
        settings.append(setting)

        setting = Setting(6, "DSC revision", self.hardware_rev[dsc], "{0:4s}")
        settings.append(setting)

        setting = Setting(7, "SYS revision", self.hardware_rev[sys], "{0:4s}")
        settings.append(setting)

        setting = Setting(8, "CPU serial number", self.hardware_sn[cpu], "{0:10s}")
        settings.append(setting)

        setting = Setting(9, "XDR serial number", self.hardware_sn[xdr], "{0:6s}")
        settings.append(setting)

        setting = Setting(10, "DSC serial number", self.hardware_sn[dsc], "{0:10s}")
        settings.append(setting)

        setting = Setting(11, "SYS serial number", self.hardware_sn[sys], "{0:6s}")
        settings.append(setting)

        setting = Setting(12, "CPU", self._get_hardware_pn_sn_string(cpu), "{0:s}")
        settings.append(setting)

        setting = Setting(13, "XDR", self._get_hardware_pn_sn_string(xdr), "{0:s}")
        settings.append(setting)

        setting = Setting(14, "DSC", self._get_hardware_pn_sn_string(dsc), "{0:s}")
        settings.append(setting)

        setting = Setting(15, "SYS", self._get_hardware_pn_sn_string(sys), "{0:s}")
        settings.append(setting)

        if not self.is_valid:
            for setting in settings:
                setting.value_string = "-"

        return settings

    def _get_hardware_pn_sn_string(self, num):
        return self.hardware_pn[num] + self.hardware_rev[num] + " / " + self.hardware_sn[num]


class SystemFeatures:
    """Class that contains system features results.
    """
    _STRUCTURE_ID = 0x26
    _VERSION = 0x10
    _SIZE = 7
    _OFFSET = 6
    _NUM_FEATURES = 2

    def __init__(self):
        self.is_valid = False
        """Defines if data in the class are valid (i.e. they are decoded properly)"""
        self.struct_id = SystemFeatures._STRUCTURE_ID
        """Structure ID number."""
        self.version = SystemFeatures._VERSION
        """Structure version number."""
        self.size = SystemFeatures._SIZE
        """Structure size."""
        self.features = _init_features(SystemFeatures._NUM_FEATURES)
        """List of features status: (1 installed, 0 not)."""

    def __str__(self):
        return (
            "System Features\n"
            "---------------\n"
            "Structure ID    : 0x{0:02X}       ({0})\n"
            "Version         : 0x{1:02X}       ({1})\n"
            "Size            : 0x{2:08X} ({2})\n"
            "Base Accuracy   : {3}\n"
            "High Accuracy   : {4}\n"
        ).format(
            self.struct_id,
            self.version,
            self.size,
            bool(self.features[0]),
            bool(self.features[1]),
        )

    def to_string(self):
        """Outputs information to string for logging
        """
        return (
            "System Features\n"
            "---------------\n"
            "Base Accuracy   : {}\n"
            "High Accuracy   : {}\n\n"
        ).format(
            bool(self.features[0]),
            bool(self.features[1]),
        )

    def decode_from_array(self, arr: bytearray):
        """Decodes system features from byte array.
        """
        length = len(arr)
        if length < SystemFeatures._SIZE + SystemFeatures._OFFSET:
            return
        arr = arr[SystemFeatures._OFFSET:]
        try:
            self.struct_id = arr[0]
            self.version = arr[1]
            [self.size] = struct.unpack("I", arr[2:6])
            self.features = []
            base = arr[6]
            self.features.append(base)
            if base == 0:
                self.features.append(1)
            else:
                self.features.append(0)
            self.is_valid = True

        except ValueError:
            self.is_valid = False

    def decode(self, packet: AppLayerPacket):
        """Decodes system features from application layer packet.
        """
        self.decode_from_array(packet.get_payload())

    @classmethod
    def encode(cls, feature_code: bytearray, cmd_id: int):
        """Encodes arr of feature codes to AppLayerPacket.
        """
        length = len(feature_code)
        arr = bytearray(length + 4)
        struct.pack_into("I", arr, 0, cmd_id)
        for i in range(length):
            arr[i+4] = feature_code[i]
        pkt = AppLayerPacket()
        pkt.create_from_payload(AppLayerIdType.CMD_BIN, arr)
        return pkt

    def get_settings(self):
        """Returns array of settings for features.
        """
        features_names = ["Base accuracy feature", "High accuracy feature"]
        settings = []
        for i in range(SystemFeatures._NUM_FEATURES):
            setting = Setting(i, features_names[i], self.features[i], "{0:d}")
            setting.value_string = _features_to_string(setting.value)
            settings.append(setting)
        return settings

def _init_features(num: int):
    """Initialize array of settings for tests
    """
    features = []
    for _ in range(num):
        features.append(0)
    return features

def _features_to_string(value: int) ->str:
    """Converts feature value to string
    """
    value_string = "-"
    if value == 1:
        value_string = "✓"
    return value_string

class SystemSetup:
    """Class that contains user system setup.
    """
    #pylint: disable=too-many-instance-attributes

    _STRUCTURE_ID = 0x22
    _VERSION = 0x11
    _SIZE = 16
    _SIZEx11 = 16+4
    _OFFSET = 6

    def __init__(self):
        self.is_valid = False
        """Defines if data in the class are valid (i.e. they are decoded properly)"""
        self.struct_id = SystemSetup._STRUCTURE_ID
        """Structure ID number."""
        self.version = SystemSetup._VERSION
        """Structure version number."""
        self.size = SystemSetup._SIZE
        """Structure size."""
        self.software_trigger = 0
        """Software trigger flag (0 - disabled, 1 - enabled)."""
        self.baud_rate_type = 0
        """Baud rate type (3 - 9600, 7 - 115200)."""
        self.speed_of_sound = 1500
        """Speed of sound in m/s."""
        self.max_depth = 60
        """Maximum tracking depth in meters."""
        self.max_vb_range = 0
        """Maximum vertical beam range in meters."""

    def __str__(self):
        return (
            "System Setup\n"
            "------------\n"
            "Structure ID     : 0x{0:02X}       ({0})\n"
            "Version          : 0x{1:02X}       ({1})\n"
            "Size             : 0x{2:08X} ({2})\n"
            "Software Trigger : {3}\n"
            "Baudrate         : {4}\n"
            "Speed of Sound   : {5}\n"
            "Max Depth        : {6}\n"
            "Max VB Range     : {7}"
        ).format(
            self.struct_id,
            self.version,
            self.size,
            self.software_trigger,
            self.baud_rate_type,
            self.speed_of_sound,
            self.max_depth,
            self.max_vb_range
        )

    def to_string(self):
        """Outputs information to string for logging
        """
        baud_rate = 115200 if self.baud_rate_type == 7 else 9600
        return (
            "System Setup\n"
            "------------\n"
            "Software trigger : {}\n"
            "Baud rate        : {} (type {})\n"
            "Speed of sound   : {:.3f}\n"
            "Max BT range     : {:.3f}\n"
            "Max VB range     : {:.3f}\n\n"
        ).format(
            self.software_trigger,
            baud_rate, self.baud_rate_type,
            self.speed_of_sound,
            self.max_depth,
            self.max_vb_range
        )
    def decode_from_array(self, arr: bytearray):
        """Decodes system setup from byte array.
        """
        length = len(arr)
        if length < SystemSetup._SIZE + SystemSetup._OFFSET:
            return
        arr = arr[SystemSetup._OFFSET:]
        try:
            self.struct_id = arr[0]
            self.version = arr[1]
            [self.size] = struct.unpack("I", arr[2:6])
            self.software_trigger = arr[6]
            self.baud_rate_type = arr[7]
            [self.speed_of_sound] = struct.unpack("f", arr[8:12])
            [self.max_depth] = struct.unpack("f", arr[12:16])
            if self.version > 0x10:
                [self.max_vb_range] = struct.unpack("f", arr[16:20])
            self.is_valid = True

        except ValueError:
            self.is_valid = False

    @classmethod
    def encode(cls, setup, cmd_id: int):
        """Encodes system setup to application layer packet.
        """
        if setup.version > 0x10:
            arr = bytearray(SystemSetup._SIZEx11+4)
        else:
            arr = bytearray(SystemSetup._SIZE+4)
        struct.pack_into("I", arr, 0, cmd_id)
        arr[4] = setup.struct_id
        arr[5] = setup.version
        struct.pack_into("I", arr, 6, setup.size)
        arr[10] = setup.software_trigger
        arr[11] = setup.baud_rate_type
        struct.pack_into("f", arr, 12, setup.speed_of_sound)
        struct.pack_into("f", arr, 16, setup.max_depth)
        pkt = AppLayerPacket()
        pkt.create_from_payload(AppLayerIdType.CMD_BIN, arr)
        return pkt

    def decode(self, packet: AppLayerPacket):
        """Decodes system setup from application layer packet.
        """
        arr = packet.get_payload()
        self.decode_from_array(arr)

    def get_settings(self):
        """Returns system setup in form of settings.
        """
        settings = []

        setting = Setting(0, "Software trigger", self.software_trigger, "{0:d}")
        setting.set_min_max(0, 1)
        settings.append(setting)

        setting = Setting(1, "Baud rate type", self.baud_rate_type, "{0:d}")
        setting.set_min_max(0, 10)
        settings.append(setting)

        setting = Setting(2, "Speed of sound", self.speed_of_sound, "{0:.0f}", "m/s")
        setting.set_min_max(1400.0, 1600.0)
        settings.append(setting)

        setting = Setting(3, "Max range", self.max_depth, "{0:.0f}", "m")
        setting.set_min_max(1, 60)
        settings.append(setting)

        return settings

    def set_from_settings(self, settings):
        """Sets system setup from settings.
        """
        if len(settings) < SetupIdType.NUM_SETTINGS.value:
            return
        self.software_trigger = settings[0].value
        self.baud_rate_type = settings[1].value
        self.speed_of_sound = settings[2].value
        self.max_depth = settings[3].value

class SystemTests:
    """Class that contains system tests results.
    """
    _STRUCTURE_ID = 0x24
    _VERSION = 0x10
    _SIZE = 12
    _STATUS_SIZE = 16
    _NUM_TESTS = 6

    def __init__(self):
        self.is_valid = False
        """Defines if data in the class are valid (i.e. they are decoded properly)"""
        self.struct_id = SystemTests._STRUCTURE_ID
        """Structure ID number."""
        self.version = SystemTests._VERSION
        """Structure version number."""
        self.size = SystemTests._SIZE
        """Structure size."""
        self.tests = _init_tests(SystemTests._NUM_TESTS)
        """List of test results."""

    def to_string(self):
        """Outputs information to string for logging
        """
        tests = self.get_settings()
        txt = "System Tests\n" + "------------\n"
        for test in tests:
            txt += "{0:20s}: {1}\n".format(test.name,\
               _test_to_string(test.value))
        return txt

    def decode(self, pkt: AppLayerPacket):
        """Decodes system tests from application layer packet.
        """
        arr = pkt.get_payload()
        self.decode_from_array(arr)

    def decode_from_array(self, arr: bytearray):
        """Decodes system tests from byte array.
        """
        length = len(arr)
        offset = 6
        if length == SystemTests._STATUS_SIZE:
            offset = 4
        arr = arr[offset:]
        try:
            self.struct_id = arr[0]
            self.version = arr[1]
            [self.size] = struct.unpack("I", arr[2:6])
            self.tests = []
            for i in range(SystemTests._NUM_TESTS):
                self.tests.append(arr[6+i])
            self.is_valid = True

        except ValueError:
            self.is_valid = False

    def get_settings(self):
        """Returns arrays of settings for tests.
        """
        test_names = ["SDRAM test", "FRAM test", "RTC test", \
        "EEPROM test", "QSPI DSC test", "ADC test"]
        settings = []
        for i in range(SystemTests._NUM_TESTS):
            setting = Setting(i, test_names[i], self.tests[i], "{0:d}")
            settings.append(setting)
        return settings

def _init_tests(num: int):
    """Initialize array of settings for tests.
    """
    tests = []
    for _ in range(num):
        tests.append(TestResultIdType.NOT_RUN.value)
    return tests

def _test_to_string(value):
    """Converts test result to string
    """
    value_string = "NOT RUN"
    if value == 1:
        value_string = "PASSED"
    elif value == 0:
        value_string = "FAILED"
    return value_string

class SystemUpdate:
    """Class that contains system firmware update logic.
    """
    _STRUCTURE_ID = 0x27
    _VERSION = 0x10
    _SIZE = 14

    def __init__(self):
        self.struct_id = SystemUpdate._STRUCTURE_ID
        """Structure ID number."""
        self.version = SystemUpdate._VERSION
        """Structure version number."""
        self.size = SystemUpdate._SIZE
        """Structure size."""

    @classmethod
    def encode(cls, file_size: int, chunk_size: int, cmd_id: int):
        """Encodes request to start firmware update to AppLayerPacket.
        """
        arr = bytearray(SystemUpdate._SIZE+4)
        struct.pack_into("I", arr, 0, cmd_id)
        arr[4] = SystemUpdate._STRUCTURE_ID
        arr[5] = SystemUpdate._VERSION
        struct.pack_into("I", arr, 6, SystemUpdate._SIZE)
        struct.pack_into("I", arr, 10, file_size)
        struct.pack_into("I", arr, 14, chunk_size)
        pkt = AppLayerPacket()
        pkt.create_from_payload(AppLayerIdType.CMD_BIN, arr)
        return pkt

    @classmethod
    def encode_data(cls, data: bytearray, cmd_id: int, chunk_size: int):
        """Encodes request to start firmware update to AppLayerPacket.
        """
        length = len(data)
        arr = bytearray(chunk_size+4)
        struct.pack_into("I", arr, 0, cmd_id)
        arr[4:4+length] = data
        pkt = AppLayerPacket()
        pkt.create_from_payload(AppLayerIdType.CMD_BIN, arr)
        return pkt

class SystemUpdateStatus:
    """Class that contains system firmware update status.
    """
    #pylint: disable=too-many-instance-attributes
    _STRUCTURE_ID = 0x28
    _VERSION = 0x10
    _SIZE = 11
    _OFFSET = 4

    def __init__(self):
        self.is_valid = False
        """Defines if data in the class are valid (i.e. they are decoded properly)"""
        self.struct_id = SystemUpdateStatus._STRUCTURE_ID
        """Structure ID number."""
        self.version = SystemUpdateStatus._VERSION
        """Structure version number."""
        self.size = SystemUpdateStatus._SIZE
        """Structure size"""
        self.monolith = 0
        """Monolith status."""
        self.fpga_image = 0
        """FPGA image status."""
        self.fpga_update = 0
        """FPGA update status."""
        self.apps_image = 0
        """Firmware image status."""
        self.apps_update = 0
        """Firmware update status."""

    def decode_from_array(self, arr: bytearray):
        """Decodes system firmware update status from byte array.
        """
        length = len(arr)
        if length < SystemUpdateStatus._SIZE + SystemUpdateStatus._OFFSET:
            return
        arr = arr[SystemUpdateStatus._OFFSET:]
        try:
            self.struct_id = arr[0]
            self.version = arr[1]
            [self.size] = struct.unpack("I", arr[2:6])
            self.monolith = arr[6]
            self.fpga_image = arr[7]
            self.fpga_update = arr[8]
            self.apps_image = arr[9]
            self.apps_update = arr[10]
            self.is_valid = True

        except ValueError:
            self.is_valid = False

    def decode(self, packet: AppLayerPacket):
        """Decodes system update from application layer packet.
        """
        arr = packet.get_payload()
        self.decode_from_array(arr)

class OutputData:
    """Class that contains output ping data.
     """

    _MIN_SIZE = 87
    _VERSION = 0x11
    COORDINATES = ["Beam", "XYZ", "Ship", "Earth"]
    """List of coordinate systems ("Beam", "XYZ", "Ship", "Earth")."""

    def __init__(self, arr: bytearray):
        if arr is None:
            self._create_empty()
            return

        length = len(arr)
        [self.size] = struct.unpack("I", arr[2:6])
        self.is_valid = (length == self.size)
        """Defines if data in the class are valid."""
        if length < OutputData._MIN_SIZE:
            self.is_valid = False
        self._checksum = self._checksum = arr[-2] | (arr[-1] << 8)
        checksum = calc_checksum(arr[0:-2])
        if self._checksum != checksum:
            self.is_valid = False
        if not self.is_valid:
            return

        self.count = 0
        """Count used externally to count number of data packets."""
        self.struct_id = arr[0]
        """Structure ID number."""
        self.version = arr[1]
        """Structure version number."""
        self.system_type = arr[6]
        """System type (76 for Wayfinder)."""
        self.system_subtype = arr[7]
        """System sub-type (0 for Wayfinder)."""
        self.fw_major_version = arr[8]
        """Firmware major number."""
        self.fw_minor_version = arr[9]
        """Firmware minor number."""
        self.fw_patch_version = arr[10]
        """Firmware patch number."""
        self.fw_build_version = arr[11]
        """Firmware build number."""
        self.year = arr[12]
        """Part of ping time stamp - year."""
        if self.year < 100:
            self.year += 2000
        self.month = arr[13]
        """Part of ping time stamp - month."""
        self.day = arr[14]
        """Part of ping time stamp - day."""
        self.hour = arr[15]
        """Part of ping time stamp - hour."""
        self.minute = arr[16]
        """Part of ping time stamp - minute."""
        self.second = arr[17]
        """Part of ping time stamp - second."""
        [self.millisecond] = struct.unpack("H", arr[18:20])
        """Part of ping time stamp - millisecond."""
        self.coordinate_system = arr[20]
        """Coordinate system (0 - 3)."""
        self.vel_x = 0
        """Beam 1 or X velocity in m/s."""
        self.vel_y = 0
        """Beam 2 or Y velocity in m/s."""
        self.vel_z = 0
        """Beam 3 or Z velocity in m/s."""
        self.vel_err = 0
        """Beam 4 or error velocity in m/s."""
        [self.vel_x, self.vel_y, self.vel_z, self.vel_err] = struct.unpack("ffff", arr[21:37])
        self.range_beam1 = 0
        """ Beam 1 range to bottom in meters."""
        self.range_beam2 = 0
        """ Beam 2 range to bottom in meters."""
        self.range_beam3 = 0
        """ Beam 3 range to bottom in meters."""
        self.range_beam4 = 0
        """ Beam 4 range to bottom in meters."""
        [self.range_beam1, self.range_beam2, self.range_beam3, self.range_beam4] \
            = struct.unpack("ffff", arr[37:53])
        self.mean_range = 0
        """ Mean range to bottom in meters."""
        [self.mean_range] = struct.unpack("f", arr[53:57])
        self.speed_of_sound = 0
        """Speed of sound used in m/s."""
        [self.speed_of_sound] = struct.unpack("f", arr[57:61])
        self.status = 0
        """Status word."""
        [self.status] = struct.unpack("H", arr[61:63])
        self.bit_count = arr[63]
        """Number built in test errors."""
        self.bit_code = arr[64]
        """Built in test error code.  For more information please refer to Wayfinder DVL guide."""
        self.voltage = 0
        """Input voltage in Volts."""
        [self.voltage] = struct.unpack("f", arr[65:69])
        self.transmit_voltage = 0
        """Transmit voltage in Volts."""
        [self.transmit_voltage] = struct.unpack("f", arr[69:73])
        self.current = 0
        """Current in Amps."""
        [self.current] = struct.unpack("f", arr[73:77])
        self.serial_number = "0"
        """Serial number of the system."""
        if self.version == 0x10:
            start = 77
        else:
            start = 83
            try:
                value = arr[77:83].decode("utf-8", errors='ignore')
            except UnicodeDecodeError:
                value = "0"
            self.serial_number = value

        self.reserved = arr[start:-2]

    def is_range_valid(self, beam=None):
        """Returns if range to bottom is valid

        Parameters
        ----------
        beam : int
            Beam number (1-4).  If None range corresponds to mean range.

        Returns
        -------
        bool
            True if value is valid, False otherwise.
        """
        valid = False
        if self.is_valid:
            if beam is None:
                valid = not math.isnan(self.mean_range)
            if isinstance(beam, int):
                if beam == 1:
                    valid = not math.isnan(self.range_beam1)
                elif beam == 2:
                    valid = not math.isnan(self.range_beam2)
                elif beam == 3:
                    valid = not math.isnan(self.range_beam3)
                elif beam == 4:
                    valid = math.isnan(self.range_beam4)
        return valid

    def is_velocity_valid(self, component=None):
        """Returns if velocity is valid

        Parameters
        ----------
        component : int
            Component number (1-4).  If None velocity validity corresponds to magnitude.

        Returns
        -------
        bool
            True if value is valid, False otherwise.
        """
        valid = False
        if self.is_valid:
            if component is None:
                valid = not(math.isnan(self.vel_x) or math.isnan(self.vel_y))
            if isinstance(component, int):
                if component == 1:
                    valid = not math.isnan(self.vel_x)
                elif component == 2:
                    valid = not math.isnan(self.vel_y)
                elif component == 3:
                    valid = not math.isnan(self.vel_z)
                elif component == 4:
                    valid = not math.isnan(self.vel_err)
        return valid

    def get_fw_version(self) ->str:
        """Gets firmware version as string
        """
        if self.is_valid:
            return "{0:d}.{1:02d}.{2:02d}.{3:02d}".format(self.fw_major_version,\
               self.fw_minor_version, self.fw_patch_version, self.fw_build_version)
        return "-"

    def get_date_time(self):
        """Gets date time
        """
        return datetime.datetime(self.year, self.month, self.day, \
            self.hour, self.minute, self.second, self.millisecond * 1000)

    def _create_empty(self):
        """Create empty structure
        """
        self.count = 0
        self.struct_id = 0
        self.version = 0
        self.system_type = 0
        self.system_subtype = 0
        self.fw_major_version = 0
        self.fw_minor_version = 0
        self.fw_patch_version = 0
        self.fw_build_version = 0
        self.year = 0
        self.month = 1
        self.day = 1
        self.hour = 0
        self.minute = 0
        self.second = 0
        self.millisecond = 0
        self.coordinate_system = 0
        self.vel_x = self.vel_y = self.vel_z = self.vel_err = 0
        self.range_beam1 = self.range_beam2 = 0
        self.range_beam3 = self.range_beam4 = 0
        self.mean_range = self.speed_of_sound = 0
        self.status = 0
        self.bit_code = 0
        self.bit_count = 0
        self.voltage = 0
        self.transmit_voltage = 0
        self.current = 0
        self.serial_number = ""
        self.reserved = []
        self.is_valid = False

    def get_settings(self):
        """Returns data in form of settings
        """
        settings = []
        settings.append(Setting(0, "Count", self.count, "{0:d}"))

        date = "{:02d}/{:02d}/{:02d}".format(self.year, self.month, self.day)
        setting = Setting(1, "Date", date, "{0:s}")
        settings.append(setting)

        time = "{:02d}:{:02d}:{:02d}.{:03d}".format(self.hour, \
            self.minute, self.second, self.millisecond)
        setting = Setting(2, "Time", time, "{0:s}")
        settings.append(setting)

        value = Setting.INVALID
        if 0 <= self.coordinate_system <= 3:
            value = OutputData.COORDINATES[self.coordinate_system]

        settings.append(Setting(3, "Coordinate", value, "{0:s}"))
        fmt = "{0:.3f}"
        setting = Setting(4, "Velocity X", self.vel_x, fmt, "m/s")
        settings.append(setting)

        setting = Setting(5, "Velocity Y", self.vel_y, fmt, "m/s")
        settings.append(setting)

        setting = Setting(6, "Velocity Z", self.vel_z, fmt, "m/s")
        settings.append(setting)

        setting = Setting(7, "Velocity Err", self.vel_err, fmt, "m/s")
        settings.append(setting)

        setting = Setting(8, "Beam 1 range", self.range_beam1, fmt, "m")
        settings.append(setting)

        setting = Setting(9, "Beam 2 range", self.range_beam2, fmt, "m")
        settings.append(setting)

        setting = Setting(10, "Beam 3 range", self.range_beam3, fmt, "m")
        settings.append(setting)

        setting = Setting(11, "Beam 4 range", self.range_beam4, fmt, "m")
        settings.append(setting)

        setting = Setting(12, "Mean range", self.mean_range, fmt, "m")
        settings.append(setting)

        setting = Setting(13, "Speed of sound", self.speed_of_sound, "{0:.1f}", "m/s")
        settings.append(setting)

        setting = Setting(14, "Input voltage", self.voltage, "{0:.1f}", "V")
        settings.append(setting)

        setting = Setting(15, "Transmit voltage", self.transmit_voltage, "{0:.1f}", "V")
        settings.append(setting)

        settings.append(Setting(16, "Current", self.current, "{0:.1f}", "A"))
        settings.append(Setting(17, "Status", self.status, "0x{:04X}"))
        settings.append(Setting(18, "BIT count", self.bit_count, "{0:d}"))
        settings.append(Setting(19, "BIT codes", self.bit_code, "0x{:02X}"))

        if not self.is_valid:
            for setting in settings:
                setting.value_string = "NO DATA"

        return settings

class FftTest:
    """Class that contains FFT (interference) test
    """
    #pylint: disable=too-many-instance-attributes
    #pylint: disable=too-few-public-methods
    _STRUCTURE_ID = 0x25
    _VERSION = 0x10
    _SIZE = 34
    _OFFSET = 6

    def __init__(self):
        self.is_valid = False
        """Defines if data in the class are valid (i.e. they are decoded properly)"""
        self.struct_id = FftTest._STRUCTURE_ID
        """Structure ID number."""
        self.version = FftTest._VERSION
        """Structure version."""
        self.size = FftTest._SIZE
        """Structure size."""
        self.samples_to_collect = 32768
        """Samples to collect."""
        self.fft_len = 1024
        """FFT length."""
        self.sample_offset = 10000
        """Sample offset."""
        self.gain = 1
        """Gain."""
        self.beam_mask = 15
        """Beam mask."""
        self.bandwidth = 1
        """Bandwidth."""
        self.system_freq = 614400
        """System frequency."""

    @classmethod
    def encode(cls, setup, cmd_id: int):
        """Encodes system setup to byte array
        """
        arr = bytearray(FftTest._SIZE+4)
        struct.pack_into("I", arr, 0, cmd_id)
        arr[4] = setup.struct_id
        arr[5] = FftTest._VERSION
        struct.pack_into("I", arr, 6, FftTest._SIZE)
        struct.pack_into("I", arr, 10, setup.samples_to_collect)
        struct.pack_into("I", arr, 14, setup.fft_len)
        struct.pack_into("I", arr, 18, setup.sample_offset)
        struct.pack_into("I", arr, 22, setup.gain)
        struct.pack_into("I", arr, 26, setup.beam_mask)
        struct.pack_into("I", arr, 30, setup.bandwidth)
        struct.pack_into("I", arr, 34, setup.system_freq)
        pkt = AppLayerPacket()
        pkt.create_from_payload(AppLayerIdType.CMD_BIN, arr)
        return pkt

class FftData:
    """Class that contains FFT (interference) test data
    """
    #pylint: disable=too-many-instance-attributes
    NUM_BEAMS = 4
    _SIZE = 32812

    def __init__(self, arr: bytearray):
        self.is_valid = False
        """Defines if data in the class are valid (i.e. they are decoded properly)"""
        self.tag = 0
        self.samples_to_collect = 0
        self.fft_len = 0
        self.sample_offset = 0
        self.gain = 0
        self.beam_mask = 0
        self.bandwidth = 0
        self.system_freq = 0
        self.sample_freq = 0
        self.gain = 0
        self.count = 0
        self.data = []
        self.xdata = []
        if arr is not None:
            self.decode_from_array(arr)

    def decode_from_array(self, arr: bytearray):
        """Decodes system tests from byte array
        """
        length = len(arr)
        if length < FftData._SIZE:
            self.__init__(None)
            return
        try:
            [self.tag] = struct.unpack("I", arr[0:4])
            [size] = struct.unpack("I", arr[4:8])
            if size != FftData._SIZE:
                return
            [self.samples_to_collect] = struct.unpack("I", arr[8:12])
            [self.fft_len] = struct.unpack("I", arr[12:16])
            [self.sample_offset] = struct.unpack("I", arr[16:20])
            [self.gain] = struct.unpack("I", arr[20:24])
            [self.beam_mask] = struct.unpack("I", arr[24:28])
            [self.bandwidth] = struct.unpack("I", arr[28:32])
            [self.system_freq] = struct.unpack("I", arr[32:36])
            [self.sample_freq] = struct.unpack("I", arr[36:40])
            samples = self.fft_len
            beams = FftData.NUM_BEAMS
            self.data = []
            for i in range(beams):
                beam_data = FftBeamData(i, samples)
                self.data.append(beam_data)

            offset = 40
            fft_len = samples
            for i in range(fft_len):
                for beam_data in self.data:
                    end = offset+8
                    [real, imag] = struct.unpack("ii", arr[offset:end])
                    offset = end
                    beam_data.signal[i] = complex(real, imag)
            self.is_valid = True
            self.count = 1
        except ValueError:
            self.is_valid = False

    def decode(self, packet: AppLayerPacket):
        """Decodes system tests from application layer packet
        """
        self.decode_from_array(packet.get_payload())

    def process(self):
        """Processes FFT data
        """
        if self.is_valid:
            base_freq = self.system_freq * 0.001
            freq_khz = base_freq
            if self.bandwidth == 1:
                freq_khz /= 4.0
            freqband = freq_khz/self.fft_len
            self.xdata = np.zeros(self.fft_len, dtype=np.float)
            for i in range(self.fft_len):
                self.xdata[i] = freqband * i - freq_khz / 2
            for beam_data in self.data:
                beam_data.do_fft()
                beam_data.frequency_peak = self.xdata[beam_data.peak_index] + base_freq

    def average(self, fft):
        """Averages FFT
        """
        if self.count == 0:
            self.xdata = fft.xdata
            self.data = fft.data
            self.system_freq = fft.system_freq
            self.bandwidth = fft.bandwidth
            self.is_valid = True

        base_freq = self.system_freq * 0.001
        for i, beam_data in enumerate(self.data):
            beam_data.average(fft.data[i], self.count)
            beam_data.frequency_peak = self.xdata[beam_data.peak_index] + base_freq
        self.count += 1

class FftBeamData:
    """Class that stores FFT beam data
    """
    #pylint: disable=too-many-instance-attributes
    def __init__(self, beam_no: int, fft_len: int):
        self.beam = beam_no
        """Beam number (0-3)"""
        self.fft_len = fft_len
        """FFT length."""
        self.signal = np.zeros(fft_len, dtype=np.complex_)
        """Original signal."""
        self.spectrum = None
        """Power spectrum."""
        self.power_ratio = None
        """Power ratio in dB."""
        self.peak_index = 0
        """Peak index."""
        self.frequency_peak = 0
        """Frequency peak in kHz."""
        self.power_peak = 0
        """Power peak in dB."""
        self.processed = False
        """Flag to indicate if data were processed."""

    def do_fft(self):
        """Performs FFT.
        """
        data = self.signal
        spectrum = np.fft.fft(data)
        mag2 = 0
        for i in range(self.fft_len):
            mag = abs(spectrum[i])
            mag2 += mag*mag

        self.spectrum = np.zeros(self.fft_len, dtype=np.float)
        half = int(self.fft_len/2)
        for i in range(half):
            k = i + half
            self.spectrum[k] = _normalize(spectrum[i], mag2)
            self.spectrum[i] = _normalize(spectrum[k], mag2)

        self.power_ratio = np.zeros(self.fft_len, dtype=np.float)
        self._calc_power()
        self.processed = True

    def _calc_power(self):
        max_spectrum = 0
        for i in range(self.fft_len):
            value = self.spectrum[i]
            if value > max_spectrum:
                max_spectrum = value
                self.peak_index = i
            if value == 0:
                self.power_ratio[i] = -50
            else:
                self.power_ratio[i] = 10 * np.log10(np.absolute(value))
        self.power_peak = self.power_ratio[self.peak_index]

    def average(self, beam_data, count: int):
        """Averages FFT data in a beam.
        """
        if self.processed:
            if count == 0:
                for i in range(self.fft_len):
                    self.spectrum[i] = beam_data.spectrum[i]
            else:
                for i in range(self.fft_len):
                    value = self.spectrum[i] * count + beam_data.spectrum[i]
                    self.spectrum[i] = value / (count + 1)
            self._calc_power()

def _normalize(value, mag: float) ->float:
    """Normalizes FFT.
    """
    norm = 0
    if mag > 0:
        avalue = abs(value)
        rnorm = avalue * avalue / mag
        norm = rnorm
    return norm

class SystemInfoIdType(Enum):
    """Enumerated type class that defines system info IDs.  Used with list of settings for display.
    """
    FREQUENCY = 0
    """Frequency ID."""
    FIRMWARE_VERSION = auto()
    """Firmware version."""
    FPGA_VERSION = auto()
    """FPGA version."""
    SYSTEM_ID = auto()
    """System ID."""


class ComponentsIdType(Enum):
    """Enumerated type class that defines hardware components IDs.
       Used with list of settings for display.
    """
    CPU_PN = 0
    """CPU part number."""
    XDR_PN = auto()
    """XDR part number."""
    DSC_PN = auto()
    """DSC part number."""
    SYS_PN = auto()
    """SYS part number."""
    CPU_REV = auto()
    """CPU revision number."""
    XDR_REV = auto()
    """XDR revision number."""
    DSC_REV = auto()
    """DSC revision number."""
    SYS_REV = auto()
    """SYS revision number."""
    CPU_SN = auto()
    """CPU serial number."""
    XDR_SN = auto()
    """XDR serial number."""
    DSC_SN = auto()
    """DSC serial number."""
    SYS_SN = auto()
    """SYS serial number."""
    CPU_PN_SN = auto()
    """CPU part and serial number."""
    XDR_PN_SN = auto()
    """XDR part and serial number."""
    DSC_PN_SN = auto()
    """DSC part and serial number."""
    SYS_PN_SN = auto()
    """SYS part and serial number."""

class FeaturesIdType(Enum):
    """Enumerated type class that defines features IDs.  Used with list of settings for display.
    """
    BASE_ACC_FEATURE = 0
    """Base accuracy feature."""
    HIGH_ACC_FEATURE = 1
    """High accuracy feature."""

class BaudRateType(Enum):
    """Enumerated type class that defines baud rate types.
    """
    BAUD_9600 = 3
    """9600 baud rate."""
    BAUD_115200 = 7
    """115200 baud rate."""

class SetupIdType(Enum):
    """Enumerated type class that defines system setup IDs. Used with list of settings for display.
    """
    SOFTWARE_TRIGGER = 0
    """Software trigger."""
    BAUD_RATE_TYPE = auto()
    """Baud rate type"""
    SPEED_OF_SOUND = auto()
    """Speed of sound."""
    MAX_RANGE = auto()
    """Maximum depth range."""
    NUM_SETTINGS = auto()
    """Number of setup settings"""

class TestsIdType(Enum):
    """Enumerated type class that defines system tests IDs.  Used with list of settings for display.
    """
    SDRAM_TEST = 0
    """SDRAM test."""
    FRAM_TEST = auto()
    """FRAM test."""
    RTC_TEST = auto()
    """Real-time clock test."""
    EEPROM_TEST = auto()
    """EEPROM test."""
    QSPI_DSC_TEST = auto()
    """QSPI/DSC test."""
    ADC_TEST = auto()
    """ADC test."""

class TestResultIdType(Enum):
    """Enumerated type class that defines system tests results IDs.
    """
    NOT_RUN = -1
    """Test was not executed yet."""
    FAILED = auto()
    """Test failed."""
    PASSED = auto()
    """Test passed."""
    RUNNING = auto()
    """Test is running."""


class DataIdType(Enum):
    """Enumerated type class that defines IDs for output data. \
   Used with list of settings for display.
    """
    COUNT = 0
    """Count of packets."""
    DATE = auto()
    """Date of the ping."""
    TIME = auto()
    """Time of the ping."""
    COORD_SYSTEM = auto()
    """Coordinate system."""
    VEL_X = auto()
    """X velocity."""
    VEL_Y = auto()
    """Y velocity."""
    VEL_Z = auto()
    """Z velocity."""
    VEL_ERR = auto()
    """Error velocity."""
    RANGE1 = auto()
    """Range to bottom beam 1."""
    RANGE2 = auto()
    """Range to bottom beam 2."""
    RANGE3 = auto()
    """Range to bottom beam 3."""
    RANGE4 = auto()
    """Range to bottom beam 4."""
    MEAN_RANGE = auto()
    """Mean range to bottom."""
    SPEED_OF_SOUND = auto()
    """Speed of sound."""
    VOLTAGE = auto()
    """Input voltage."""
    TRANSMIT_VOLTAGE = auto()
    """Transmit voltage."""
    CURRENT = auto()
    """Transmit current."""
    STATUS = auto()
    """Ping status."""
    BIT_COUNT = auto()
    """Number of built in tests."""
    BIT_CODE = auto()
    """Built in test code."""
