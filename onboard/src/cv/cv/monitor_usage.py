import rclpy
from rclpy.node import Node
import subprocess
import psutil
from std_msgs.msg import String
import re

#create the new node
class ResourceMonitor(Node):
    def __init__(self):
        super().__init__('resource_monitor')

        #create publisher so it can send messaages to the topic node_resources
        self.publisher_ = self.create_publisher(String, 'node_resources', 10)

        #run the check_resources() function every 2 seconds
        self.timer = self.create_timer(2.0, self.check_resources)
        self.get_logger().info('Resource monitor started (CPU, MEM, GPU, TEMP).')

    def get_ros_nodes(self):
        """Get list of active ROS2 nodes."""
        try:
            result = subprocess.run(['ros2', 'node', 'list'], capture_output=True, text=True, check=True)
            return result.stdout.strip().splitlines()
        except subprocess.CalledProcessError:
            return []

    #need the process IDs for nodes, since OS organizes by PID, not "node name"
    #therefore need the PID to be able to track resource usage
    def find_pid_for_node(self, node_name):
        """Find PIDs of processes running this node (heuristic)."""
        try:
            result = subprocess.run(["ps", "-eo", "pid,cmd"], capture_output=True, text=True)
            pids = []
            for line in result.stdout.splitlines():
                parts = line.strip().split(None, 1)
                if len(parts) == 2:
                    pid, cmd = parts
                    if node_name.lstrip('/') in cmd:
                        pids.append(int(pid))
            return pids
        except Exception:
            return []

    #tegrastats = built in command for Jetson nano that shows hardware stats
    #uses regex to extract GPU usage/temp, RAM, CPU temp
    def parse_tegrastats(self):
        """Run tegrastats once and extract GPU, memory, and temperature data."""
        try:
            # Collect one sample every 1 second
            output = subprocess.check_output(['tegrastats', '--interval', '1000', '--count', '1'], text=True)
            print(f"OUTPUT: {output}")
            data = {}

            # GPU usage percentage and frequency
            gpu_match = re.search(r'GR3D_FREQ (\d+)%@(\d+)', output)
            if gpu_match:
                data['gpu_usage'] = int(gpu_match.group(1))
                data['gpu_freq'] = int(gpu_match.group(2))

            # RAM usage and total in MB
            ram_match = re.search(r'RAM (\d+)/(\d+)MB', output)
            if ram_match:
                data['ram_used'] = int(ram_match.group(1))
                data['ram_total'] = int(ram_match.group(2))

            # CPU temperature
            cpu_temp_match = re.search(r'CPU@([\d\.]+)C', output)
            if cpu_temp_match:
                data['cpu_temp'] = float(cpu_temp_match.group(1))

            # GPU temperature
            gpu_temp_match = re.search(r'GPU@([\d\.]+)C', output)
            if gpu_temp_match:
                data['gpu_temp'] = float(gpu_temp_match.group(1))

            # Power draw in mW
            power_match = re.search(r'POM_5V_IN (\d+)mW', output)
            if power_match:
                data['power_mW'] = int(power_match.group(1))

            return data
        except Exception as e:
            self.get_logger().warn(f"tegrastats read failed: {e}")
            return {}


    def check_resources(self):
        nodes = self.get_ros_nodes()
        tegra = self.parse_tegrastats()

        #building a status message with all the usage info of the nodes
        msg = ""
        msg += f"\n=== System ===\n"
        print(f'TEGRA {tegra}')
        exit(0)
        if tegra:
            msg += f"GPU: {tegra.get('gpu_usage', 'N/A')}% @ {tegra.get('gpu_freq', 'N/A')} MHz | "
            msg += f"RAM: {tegra.get('ram_used', 'N/A')}/{tegra.get('ram_total', 'N/A')} MB | "
            msg += f"T(CPU): {tegra.get('cpu_temp', 'N/A')}°C | T(GPU): {tegra.get('gpu_temp', 'N/A')}°C | "
            msg += f"Power: {tegra.get('power_mW', 'N/A')} mW\n"

        msg += "=== Node Resource Usage ===\n"
        for node in nodes:
            pids = self.find_pid_for_node(node)
            for pid in pids:
                try:
                    proc = psutil.Process(pid)

                    #changed interval to 0.1, otherwise cpu_percent might always return 0.0
                    cpu = proc.cpu_percent(interval=0.1)
                    mem = proc.memory_info().rss / (1024*1024)
                    msg += f"{node} (PID {pid}): CPU={cpu:.1f}%, MEM={mem:.1f}MB\n"
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    continue

        #publishes the messages to the node_resources topic
        #so other ROS nodes can subscribe to it
        if msg:
            self.get_logger().info(msg)
            self.publisher_.publish(String(data=msg))

#running the node: start the ROS system, create ResourceMonitor, and keep alive until u stop
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
