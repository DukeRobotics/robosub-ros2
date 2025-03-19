import os
import time

import numpy as np
import rclpy
import serial
from std_msgs.msg import Float64

from offboard_comms.serial_node import SerialNode, SerialReadType


class GyroPublisher(SerialNode):
    """A class to read and publish raw gyro data from a serial port."""

    SERIAL_DEVICE_NAME = 'Gyro'
    CONFIG_FILE_PATH = f'package://offboard_comms/config/{os.getenv("ROBOT_NAME")}.yaml'

    BAUDRATE = 460800
    NODE_NAME = 'gyro_pub'
    ANGULAR_VELOCITY_TOPIC_NAME = 'sensors/gyro/angular_velocity'
    ANGULAR_POSITION_TOPIC_NAME = 'sensors/gyro/angular_position'
    TEMPERATURE_TOPIC_NAME = 'sensors/gyro/temperature'
    LINE_DELIM = ','

    CONNECTION_RETRY_PERIOD = 1.0 #S
    LOOP_RATE = 1000.0 #Hz
    TRIGGER_RATE = 1000.0 #Hz

    FRAME_START_BYTE = 0x80
    FRAME_NUM_BYTES = 10

    SCALE_FACTOR = 15143697.0

    def __init__(self) -> None:

        super().__init__(self.NODE_NAME, self.BAUDRATE, self.CONFIG_FILE_PATH, self.SERIAL_DEVICE_NAME,
                         SerialReadType.BYTES_ALL, self.CONNECTION_RETRY_PERIOD, self.LOOP_RATE,
                         parity=serial.PARITY_EVEN, read_timeout=0, num_bytes_to_read=self.FRAME_NUM_BYTES,
                         flush_input_after_read=True)

        self.debug = self.declare_parameter('debug', False).value

        self.trigger_timer = self.create_timer(1.0 / self.TRIGGER_RATE, self.trigger_callback)

        self.line_num = 1
        self.last_start_line_num = 0

        self.buffer = bytearray()

        self.angular_velocity_publisher = self.create_publisher(Float64, self.ANGULAR_VELOCITY_TOPIC_NAME, 10)
        self.angular_position_publisher = self.create_publisher(Float64, self.ANGULAR_POSITION_TOPIC_NAME, 10)
        self.temperature_publisher = self.create_publisher(Float64, self.TEMPERATURE_TOPIC_NAME, 10)

        self.angular_velocity_msg = Float64()
        self.angular_position_msg = Float64()
        self.temperature_msg = Float64()


    def get_ftdi_string(self) -> str:
        """
        Get the FTDI string for the Gyro.

        Returns:
            str: FTDI string for the Gyro.
        """
        return self._config['gyro']['ftdi']

    def process_bytes(self, data: bytes) -> None:
        """
        Process the data from the gyro.

        Args:
            data (bytes): data to process
        """
        self.buffer.extend(bytearray(data))
        self.process_buffer()

        # Print line num with width of 4, then bytes as zeroes and ones. If byte is 0x80, then add " - START" to the end
        if self.debug:
            line = f'{self.line_num:05d} ' + ' '.join(format(byte, '08b') for byte in data)
            if data[0] == self.FRAME_START_BYTE:
                line += f' - START {self.line_num - self.last_start_line_num}'
                self.last_start_line_num = self.line_num

            self.get_logger().info(line)
            self.line_num += 1

    def process_buffer(self) -> None:
        """Process the buffer of data from the gyro."""
        index = 0
        clear_buffer_up_to_index = 0
        while index < len(self.buffer):
            if self.buffer[index] == self.FRAME_START_BYTE:
                if index + self.FRAME_NUM_BYTES > len(self.buffer):
                    clear_buffer_up_to_index = index
                    break
                self.process_frame(index)
                index += self.FRAME_NUM_BYTES
                clear_buffer_up_to_index = index
            else:
                index += 1

        # Remove all bytes up to but not including the clear_buffer_up_to_index
        self.buffer = self.buffer[clear_buffer_up_to_index:]

    def process_frame(self, start_byte_index: int) -> None:
        """
        Process a frame of data from the gyro.

        Verify the checksums and publish the angular velocity and temperature.

        Args:
            start_byte_index (int): Index of the start byte in the buffer. The buffer must have at least FRAME_NUM_BYTES
                bytes in it, starting from this index.
        """
        # Generate two checksums
        # First checksum is XOR of second through sixth bytes
        # Second checksum is XOR of second through ninth bytes
        checksum1 = 0
        checksum2 = 0
        for i in range(1, 9):
            if i < 6:  # noqa: PLR2004
                checksum1 ^= self.buffer[start_byte_index + i]
            checksum2 ^= self.buffer[start_byte_index + i]

        # Check if checksums are valid
        if checksum1 != self.buffer[start_byte_index + 6] or checksum2 != self.buffer[start_byte_index + 9]:
            self.get_logger().info(f'Checksum error: {checksum1} {self.buffer[start_byte_index + 6]} '
                                   f'{checksum2} {self.buffer[start_byte_index + 9]}')
            return

        gyro_data = np.int32(self.buffer[start_byte_index + 1] & 0x7F)
        gyro_data |= np.int32((self.buffer[start_byte_index + 2] & 0x7F) << 7)
        gyro_data |= np.int32((self.buffer[start_byte_index + 3] & 0x7F) << 14)
        gyro_data |= np.int32((self.buffer[start_byte_index + 4] & 0x7F) << 21)
        gyro_data |= np.int32((self.buffer[start_byte_index + 5] & 0x0F) << 28)

        temp_data = np.int16(self.buffer[start_byte_index + 7] & 0x7F)
        temp_data |= np.int16((self.buffer[start_byte_index + 8] & 0x7F) << 7)

        angular_velocity = gyro_data * self.LOOP_RATE / self.SCALE_FACTOR
        temperature = temp_data * 0.0625

        self.angular_velocity_msg.data = angular_velocity
        self.angular_velocity_publisher.publish(self.angular_velocity_msg)

        self.angular_position_msg.data += angular_velocity / self.LOOP_RATE
        self.angular_position_publisher.publish(self.angular_position_msg)

        self.temperature_msg.data = temperature
        self.temperature_publisher.publish(self.temperature_msg)

        if self.debug:
            self.get_logger().info(f'Angular velocity: {angular_velocity:.9f} deg/s, Temperature: {temperature:.2f} C')


    def trigger_callback(self) -> None:
        """Trigger the gyro to send data."""
        if self._serial and self._serial.is_open:
            self.writebytes(b'1')
            time.sleep((1.0 / self.TRIGGER_RATE) / 2.0)  # Sleep for half the trigger period (50% duty cycle)
            self.writebytes(b'0')  # Send the stop trigger


def main(args: list[str] | None = None) -> None:
    """Create and run the gyro publisher node."""
    rclpy.init(args=args)
    gyro_raw = GyroPublisher()

    try:
        rclpy.spin(gyro_raw)
    except KeyboardInterrupt:
        pass
    finally:
        gyro_raw.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
