import rclpy
from rclpy.node import Node as RosNode
from std_msgs.msg import *

from nodes.config import NodeConfig
from nodes.decorators import decorated_functions
from nodes.plugin import dynamic_import

class Node(RosNode):

    def __init__(self, config):
        super().__init__(config.name)
        self._config = config
        self.get_logger().info(f"Starting {config.name}...")
        dynamic_import(NodeConfig.standard_project_dir, config.implementation)

        self._publishers = {}

        #decorated_functions
        #self.publisher_ = self.create_publisher(Int8, f"{config.name}/", 10)

def main():
    rclpy.init()
    rclpy.spin(Node(NodeConfig.from_standard()))
    rclpy.shutdown()

if __name__ == '__main__':
    main()
