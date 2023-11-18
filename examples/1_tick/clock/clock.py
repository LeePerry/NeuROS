# Copyright (c) 2023 Lee Perry

"""
This example demonstrates an extremely simple tick-based hook.
"""

import time

from neuros.hooks import neuros_tick

@neuros_tick(seconds=1.0)
def seconds_hand(node):
    """
    This hook is invoked every second and prints either "tick" or "tock" to
    the console via the standard logging system.
    """
    gmt = time.gmtime()
    sound = ["Tick", "Tock"][gmt.tm_sec % 2]
    current_time = time.strftime("%H:%M:%S", gmt)
    node.get_ros_node().get_logger().info(f"{sound}: {current_time}")
