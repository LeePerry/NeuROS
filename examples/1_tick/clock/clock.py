# Copyright (c) 2023 Lee Perry

import time

from neuros.hooks import neuros_tick

@neuros_tick(seconds=1.0)
def count_seconds(node):
    gmt = time.gmtime()
    sound = ["Tick", "Tock"][gmt.tm_sec % 2]
    current_time = time.strftime("%H:%M:%S", gmt)
    node.get_ros_node().get_logger().info(f"{sound}: {current_time}")
