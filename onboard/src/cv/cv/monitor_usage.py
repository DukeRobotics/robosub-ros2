import rclpy
from rclpy.node import Node
import subprocess
import psutil
from std_msgs.msg import String

class ResourceMonitor(Node):
    def __init__(self):
        super().__init__('resource_monitor')
        self.publisher_ = self.create_publisher(String, 'node_resources', 10)
        self.timer = self.create_timer(2.0, self.check_resources)

    def get_ros_nodes(self):
        """Get list of active ROS2 nodes."""
        try:
            result = subprocess.run(["ros2", "node", "list"], capture_output=True, text=True, check=True)
            return result.stdout.strip().splitlines()
        except:
            return []

    def find_pid_for_node(self, node_name):
        """Find PIDs of processes running this node (heuristic)."""
        try:
            result = subprocess.run(["ps", "-eo", "pid,cmd"], capture_output=True, text=True)
            pids = []
            for line in result.stdout.splitlines():
                pid, cmd = line.strip().split(None, 1)
                if node_name.lstrip('/') in cmd:
                    pids.append(int(pid))
            return pids
        except:
            return []

    def check_resources(self):
        nodes = self.get_ros_nodes()
        msg = ""
        for node in nodes:
            pids = self.find_pid_for_node(node)
            for pid in pids:
                try:
                    proc = psutil.Process(pid)
                    cpu = proc.cpu_percent(interval=None)
                    mem = proc.memory_info().rss / (1024*1024)
                    msg += f"{node} (PID {pid}): CPU={cpu:.1f}%, MEM={mem:.1f}MB\n"
                except:
                    continue
        if msg:
            self.get_logger().info("\n" + msg)
            self.publisher_.publish(String(data=msg))

def main(args=None):
    rclpy.init(args=args)
    node = ResourceMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
