# Copyright (c) 2023 Lee Perry

import os
import random
import time

from neuros.hooks import neuros_initialise, neuros_function, Optional

@neuros_initialise(outputs=Optional("ball"))
def serve(node):
    if os.environ.get("NEUROS_FIRST_TO_SERVE"):
        node.get_ros_node().get_logger().info("Serves")
        return node.make_packet("ball")

@neuros_function(inputs="ball", outputs="ball")
def hit_the_ball(node, ball):
    time.sleep(0.5)
    node.get_ros_node().get_logger().info(
        random.choice(["Whack", "Argh", "Ugh", "Smack"]))
    return ball
