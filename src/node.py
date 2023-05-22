import rclpy

class Node(rclpy.node.Node):

    def __init__(self, config, name):
        self.config = config
        self.name = name
        super().__init__(name)
        self.get_logger().info(f"Starting {name}...")

    def run(self, args=None):
        rclpy.init(args=args)
        rclpy.spin(self)
        rclpy.shutdown()
