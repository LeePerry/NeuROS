import time

from neuros.hooks import neuros_tick

@neuros_tick(1.0)
def count_seconds(node):
    gmt = time.gmtime()
    sound = ["Tick", "Tock"][gmt.tm_sec % 2]
    formatted = time.strftime("%H:%M:%S", gmt)
    node.get_logger().info(f"{sound}: {formatted}")
