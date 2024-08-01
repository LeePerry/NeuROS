#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class RawTick(Node):

    def __init__(self):
        super().__init__('clock')
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info("T_ck")

def main():
    rclpy.init()
    node = RawTick()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    rclpy.try_shutdown()

if __name__ == '__main__':
    main()
