# Copyright (c) 2023 Lee Perry

import random
import time

from neuros.hooks import neuros_initialise, neuros_function, Optional

@neuros_initialise(outputs=Optional("ball"))
def start_the_game(node):
    node.get_logger().info(f"{node.get_name()} has arrived on court")
    time.sleep(0.5)
    if node.get_neuros_parameter("first_to_serve"):
        node.get_logger().info("Serves")
        return node.make_neuros_packet("ball")

@neuros_function(inputs="ball", outputs="ball")
def whack_the_ball(node, ball):
    time.sleep(0.5)
    node.get_logger().info(random.choice(["Whack", "Argh", "Ugh", "Smack"]))
    return ball
