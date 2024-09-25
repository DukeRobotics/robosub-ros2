import rclpy
import threading
import numpy as np
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

class DRCNode(Node):

    def __init__(self, NODE_NAME, add_to_static_threads=False):
        super().__init__(NODE_NAME)

        if(add_to_static_threads):
            executor = SingleThreadedExecutor()
            executor.add_node(self)
            executor_thread = threading.Thread(target=self.executor.spin, daemon=True)
            executor_thread.start()

