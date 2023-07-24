# Copyright (c) 2023 Lee Perry

import random

from neuros.hooks import neuros_initialise, neuros_function, Optional

def request_random_level(node):
    level = node.make_packet("request")
    level.data = random.choice([0, 1, 2, 3])
    node.get_ros_node().get_logger().info(f"Requesting level {level.data}")
    node.set_user_data(level.data)
    return level

@neuros_initialise(outputs="request")
def initialise_controller(node):
    return request_random_level(node)

@neuros_function(inputs="level", outputs=Optional("request"))
def next_level(node, level):
    node.get_ros_node().get_logger().info(f"Wish I was at {level.data}")
    if level.data == node.get_user_data():
        return request_random_level(node)
