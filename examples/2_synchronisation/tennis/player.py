# Copyright (c) 2023 Lee Perry

"""
This example simulates a basic tennis match, by passing a packet of data from
one player node to another. Both player nodes utilise the same plugin code,
with slightly varying behaviour configured via an environment variable.
"""

import os
import random
import time

from neuros.hooks import neuros_initialise, neuros_function, Optional

@neuros_initialise(outputs=Optional("ball"))
def serve(node):
    """
    This hook is invoked by both player nodes on startup. Since only one player
    has the NEUROS_FIRST_TO_SERVE environment variable injected, only one ball
    packet is created and sent to the other player, thus starting the game.
    """
    if os.environ.get("NEUROS_FIRST_TO_SERVE"):
        node.get_ros_node().get_logger().info("Serves")
        return node.make_packet("ball")

@neuros_function(inputs="ball", outputs="ball")
def hit_the_ball(node, ball):
    """
    This hook is invoked when either player receives a ball. When invoked the
    hook sleeps for half a second before returning the ball to the other
    player. We can monitor the progress of the ball between players via the
    logging system.
    """
    time.sleep(0.5)
    noise = random.choice(["Whack", "Argh", "Ugh", "Smack"])
    node.get_ros_node().get_logger().info(noise)
    return ball
