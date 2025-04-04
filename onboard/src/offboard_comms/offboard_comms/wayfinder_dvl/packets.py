"""Contains several classes that implement binary packet interface.
"""
from enum import Enum
from dvl.util import print_bytes, print_bytearray, indent_string

class AppLayerIdType(Enum):
    """Enumerated type class that defines IDs for application layer packets.
    """
    UNKNOWN = 0xFF
    """Unknown command."""
    CMD_2LETTER = 0x01
    """Two letter command."""
    RSP_2LETTER = 0x02
    """Two letter response."""
    CMD_BIN = 0x03
    """Binary command."""
    RSP_BIN = 0x04
    """Binary response."""
    DATA_PD = 0x05
    """Binary data."""
    DATA_NMEA = 0x06
    """NMEA data."""
    STATUS = 0x07
    """Status response."""
    FFT_DATA = 0x08
    """FFT data."""

class AppLayerPacket:
    """Application layer communications interface.
    """
    _PACKET_VER = 2
    _MIN_PACKET_LEN = 4

    def __init__(self, arr=None):
        self.ver = 0
        self.pkt_id = 0
        self.len = 0
        self.payload = None
        if arr is not None and len(arr) > AppLayerPacket._MIN_PACKET_LEN:
            self.create_from_array(arr)
        else:
            self.create_empty()

    def __str__(self):
        return (
            "Application Packet\n"
            "------------------\n"
            "Version         : 0x{0:02X}   ({0})\n"
            "Packet ID       : 0x{1:02X}   ({1})\n"
            "Length          : 0x{2:04X} ({2})\n"
            "Payload         : \n"
            "{3}"
        ).format(
            self.ver,
            self.pkt_id.value,
            self.len,
            print_bytes(self.payload)
        )

    def get_payload(self):
        """Returns payload of application layer packet
        """
        if self.payload is None:
            return b""
        return bytes(self.payload)

    def get_bytes(self):
        """Returns byte array that corresponds to application layer packet
        """
        if self.pkt_id is None:
            return b""
        arr = bytearray(self.len)
        arr[0] = self.ver
        arr[1] = self.pkt_id.value
        arr[2] = self.len & 0x00FF
        arr[3] = (self.len & 0xFF00) >> 8
        arr[4:] = self.payload
        return bytes(arr)

    def create_empty(self):
        """Creates empty packet
        """
        self.ver = None
        self.pkt_id = None
        self.len = 0
        self.payload = None

    def create_from_array(self, arr: bytearray) ->bytearray:
        """Creates application layer packet from byte array
        """
        length = len(arr)
        if length <= AppLayerPacket._MIN_PACKET_LEN:
            self.create_empty()
        else:
            self.ver = arr[0]
            if AppLayerIdType.CMD_2LETTER.value <= arr[1] <= \
                AppLayerIdType.FFT_DATA.value:
                self.pkt_id = AppLayerIdType(arr[1])
            else:
                self.pkt_id = AppLayerIdType.UNKNOWN
            self.len = arr[2] | (arr[3] << 8)
            self.payload = arr[4:]

    def create_from_payload(self, packet_id: int, payload: bytearray):
        """Creates application layer packet from payload
        """
        length = len(payload)
        self.ver = AppLayerPacket._PACKET_VER
        self.pkt_id = packet_id
        self.len = length + AppLayerPacket._MIN_PACKET_LEN
        self.payload = payload

START_OF_PACKET = 0xAA
class PhysicalLayerPacket():
    """Physical layer communications interface
    """
    #pylint: disable=too-many-instance-attributes
    PACKET_VER = 0x10
    """Packet version number."""
    PACKET_ID = 0x01
    """Packet ID number."""
    MIN_PACKET_LEN = 7
    """Minimum packet length."""
    CHECKSUM_LENGTH = 2
    """Checksum length."""

    def __init__(self,
                 payload: AppLayerPacket = None,
                 packet_id: int = PACKET_ID,
                 version: int = PACKET_VER):
        self._version = version
        self._packet_id = packet_id
        self._length = None
        self._payload = payload
        self._checksum = None
        self._update_length()
        self._update_checksum()

    def __str__(self):
        return (
            "Raw Data        :\n"
            "{0}\n"
            "Physical Packet\n"
            "---------------\n"
            "Start of Packet : 0x{1:02X}   ({1})\n"
            "Version         : 0x{2:02X}   ({2})\n"
            "Packet ID       : 0x{3:02X}   ({3})\n"
            "Length          : 0x{4:04X} ({4})\n"
            "Payload         : \n"
            "{5}\n"
            "Checksum        : 0x{6:04X} ({6})\n"
        ).format(
            print_bytearray(self.encode()),
            START_OF_PACKET,
            self._version,
            self._packet_id,
            self._length,
            indent_string(str(self.payload)),
            self._checksum
        )

    def _calculate_payload_length(self) -> int:
        return self._payload.len if self.payload else 0

    def _update_length(self) -> None:
        self._length = self._calculate_payload_length() + PhysicalLayerPacket.MIN_PACKET_LEN

    def _to_bytearray(self, checksum_included: bool = True) -> bytearray:
        length = self.length
        if not checksum_included:
            length -= PhysicalLayerPacket.CHECKSUM_LENGTH
        array = bytearray(length)
        array[0] = START_OF_PACKET
        array[1] = self._version
        array[2] = self._packet_id
        array[3] = self.length & 0x00FF
        array[4] = (self.length & 0xFF00) >> 8
        if self._payload:
            array[5:5+self._calculate_payload_length()] = self._payload.get_bytes()
        if checksum_included:
            self._update_checksum()
            array[-2] = self.checksum & 0x00FF
            array[-1] = (self.checksum & 0xFF00) >> 8
        return array

    def _update_checksum(self) -> None:
        data = self._to_bytearray(checksum_included=False)
        self._checksum = calc_checksum(data)

    @property
    def version(self) -> int:
        """Physical layer packet version.

        Returns
        -------
        int
            The version of the communication protocol being used.
        """
        return self._version

    @version.setter
    def version(self, version: int):
        self._version = version
        self._update_checksum()

    @property
    def packet_id(self) -> int:
        """Physical layer packet ID.

        Returns
        -------
        int
            Physical layer packet ID.
        """
        return self._packet_id

    @packet_id.setter
    def packet_id(self, packet_id: int):
        self._packet_id = packet_id
        self._update_checksum()

    @property
    def length(self) -> int:
        """The length of the physical layer packet minus the checksum.

        Returns
        -------
        int
            The packet length.
        """
        return self._length

    @property
    def payload(self) -> AppLayerPacket:
        """The application layer packet encoded into a bytearray.

        Returns
        -------
        bytearray
            The application layer packet encoded into a bytearray.
        """
        return self._payload

    @payload.setter
    def payload(self, payload: AppLayerPacket):
        self._payload = payload
        self._update_length()
        self._update_checksum()

    @property
    def checksum(self) -> int:
        """The checksum of the physical layer packet.

        Returns
        -------
        int
            The checksum of the physical layer packet.
        """
        self._update_checksum()
        return self._checksum

    def encode(self) -> bytearray:
        """Converts the packet into its bytearray representation.

        Returns
        -------
        bytearray
            The bytearray that represents the physical layer packet.
        """
        data = self._to_bytearray()
        return bytes(data)


def decode(packet: bytearray) -> PhysicalLayerPacket:
    """Convert a bytearray that represents a physical layer packet into a
    PhysicalLayerPacket object.

    Parameters
    ----------
    packet : bytearray
        A bytearray that represents a physical layer packet.

    Returns
    -------
    PhysicalLayerPacket
        A object that represents the original bytearray.
    """
    if len(packet) <= PhysicalLayerPacket.MIN_PACKET_LEN or packet[0] != START_OF_PACKET:
        return None
    return PhysicalLayerPacket(
        version=packet[1],
        packet_id=packet[2],
        payload=AppLayerPacket(packet[5:-2]),
    )


def calc_checksum(arr: bytearray) -> int:
    """Calculates a 2 byte sum of the byte array ignoring rollover.

    Parameters
    ----------
    arr : bytearray
        The full range of bytes that the checksum should be calculated on.

    Returns
    -------
    int
        The checksum of the bytes.
    """
    return sum(arr) & 0xFFFF

class DecoderState(Enum):
    """Enumerated type class that defines state of the decoder.
    """
    START_BYTE = 1
    """Searching for packet start byte."""
    VER_BYTE = 2
    """Searching for packet version number."""
    PKT_ID = 3
    """Searching for packet ID."""
    PKT_LEN = 4
    """Searching for packet length."""
    PAYLOAD = 5
    """Searching for payload."""
    CHECKSUM = 6
    """Searching for checksum."""

class PacketDecoder:
    """Finds and decodes physical layer packets.
    """
    #pylint: disable=too-many-instance-attributes
    _MAX_PKT_LEN = 33000

    def __init__(self):
        self._state = DecoderState.START_BYTE
        self._packet_len = 0
        self._payload_len = 0
        self._counter = 0
        self._arr = bytearray()
        self._func_dic = { \
            DecoderState.START_BYTE : self.on_start,
            DecoderState.VER_BYTE : self.on_ver,
            DecoderState.PKT_ID : self.on_id,
            DecoderState.PKT_LEN : self.on_len,
            DecoderState.PAYLOAD : self.on_payload,
            DecoderState.CHECKSUM : self.on_checksum}

    def parse_bytes(self, arr):
        """Parses bytes array.
        """
        packets = []
        for i in range(len(arr)):
            pkts = self.parse_byte(arr[i:i+1])
            if pkts is not None:
                for pkt in pkts:
                    packets.append(pkt)
                    self.clear()
        return packets

    def clear(self):
        """Clears decoder state.
        """
        self._state = DecoderState.START_BYTE
        self._packet_len = 0
        self._payload_len = 0
        self._counter = 0
        self._arr = bytearray()

    def on_start(self, byte):
        """Searches for start of the packet.
        """
        if byte[0] == START_OF_PACKET:
            self.clear()
            self._arr += byte
            self._state = DecoderState.VER_BYTE

    def on_ver(self, byte):
        """Decodes version.
        """
        if byte[0] != PhysicalLayerPacket.PACKET_VER:
            return self.restart()
        self._state = DecoderState.PKT_ID
        return None

    def on_id(self, byte):
        """Decodes packet ID.
        """
        if byte[0] != PhysicalLayerPacket.PACKET_ID:
            return self.restart()
        self._state = DecoderState.PKT_LEN
        self._counter = 0
        return None

    def on_len(self, byte):
        """Decodes packet length.
        """
        self._packet_len |= byte[0] << (8 * self._counter)
        self._counter += 1
        if self._counter >= 2:
            min_len = PhysicalLayerPacket.MIN_PACKET_LEN
            max_len = PacketDecoder._MAX_PKT_LEN
            if self._packet_len > max_len or self._packet_len <= min_len:
                return self.restart()
            self._state = DecoderState.PAYLOAD
            self._counter = 0
            self._payload_len = self._packet_len - PhysicalLayerPacket.MIN_PACKET_LEN
        return None

    def on_payload(self, byte):
        """Decodes payload.
        """
        del byte
        self._counter += 1
        if self._counter >= self._payload_len:
            self._counter = 0
            self._state = DecoderState.CHECKSUM

    def on_checksum(self, byte):
        """Decodes and checks checksum.
        """
        del byte
        if self._counter == 0:
            self._counter += 1
        else:
            checksum = self._arr[-2] | (self._arr[-1] << 8)
            pkt = decode(self._arr)
            if pkt is not None and pkt.checksum == checksum:
                return [pkt]
            print("Checksum failed")
            return self.restart()

        return None

    def restart(self):
        """Restarts search.
        """
        self._state = DecoderState.START_BYTE
        if len(self._arr) > 1:
            arr = self._arr[1:]
            return self.parse_bytes(arr)
        return None

    def parse_byte(self, byte):
        """Parses one byte.
        """
        assert len(byte) == 1
        self._arr += byte
        return self._func_dic[self._state](byte)
