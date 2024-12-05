#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import GPUtil
import psutil
from custom_msgs.msg import SystemUsage


class SystemInfoPublisher(Node):
    TOPIC_NAME = 'system/usage'
    NODE_NAME = 'system_usage_publisher'

    def __init__(self):
        super().__init__(self.NODE_NAME)

        self._pub = self.create_publisher(SystemUsage, self.TOPIC_NAME, 10)
        self._current_msg = SystemUsage()
        self.run()

        self.get_logger().info("System info publisher node started.")

    def get_cpu(self):
        self._current_msg.cpu_percent = float(psutil.cpu_percent(interval=0.5))
        self._current_msg.cpu_speed = psutil.cpu_freq().current if psutil.cpu_freq() else 0.0

    def get_gpu(self):
        GPUs = GPUtil.getGPUs()
        if len(GPUs) > 0:
            gpu = GPUs[0]
            self._current_msg.gpu_memory.used = gpu.memoryUsed / 1000
            self._current_msg.gpu_memory.total = gpu.memoryTotal / 1000
            self._current_msg.gpu_memory.percentage = gpu.memoryUsed / gpu.memoryTotal * 100
            self._current_msg.gpu_percent = gpu.load * 100
            self._current_msg.gpu_speed = 0.0
        else:
            self._current_msg.gpu_memory.used = 0.0
            self._current_msg.gpu_memory.total = 0.0
            self._current_msg.gpu_memory.percentage = 0.0
            self._current_msg.gpu_percent = 0.0
            self._current_msg.gpu_speed = 0.0

    def get_ram(self):
        self._current_msg.ram.used = (psutil.virtual_memory().total - psutil.virtual_memory().available) / (10**9)
        self._current_msg.ram.total = psutil.virtual_memory().total / (10**9)
        self._current_msg.ram.percentage = psutil.virtual_memory().percent

    def get_disk(self):
        self._current_msg.disk.used = float(psutil.disk_usage('/').used / (10**9))
        self._current_msg.disk.total = float(psutil.disk_usage('/').total * 0.95 / (10**9))
        self._current_msg.disk.percentage = float(psutil.disk_usage('/').percent)

    def run(self):
        def timer_callback():
            if not rclpy.ok():
                timer.cancel()

            self._current_msg = SystemUsage()
            self.get_cpu()
            self.get_gpu()
            self.get_ram()
            self.get_disk()

            self._pub.publish(self._current_msg)

        timer = self.create_timer(1/15.0, timer_callback)

def main(args=None):
    rclpy.init(args=args)
    system_info_pub = SystemInfoPublisher()

    try:
        rclpy.spin(system_info_pub)
    except KeyboardInterrupt:
        pass
    finally:
        system_info_pub.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()