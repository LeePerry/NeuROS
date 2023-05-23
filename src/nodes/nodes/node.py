import rclpy
from rclpy.node import Node as RosNode

from nodes.config import NodeConfig

class Node(RosNode):

    def __init__(self, config):
        super().__init__(config.name)
        self.config = config
        self.get_logger().info(f"Starting {config.name}...")

def main():
    rclpy.init()
    rclpy.spin(Node(NodeConfig.from_standard()))
    rclpy.shutdown()

if __name__ == '__main__':
    main()
