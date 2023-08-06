# Copyright (c) 2023 Lee Perry

import os

from neuros.hooks import neuros_initialise, neuros_function, Optional
from neuros.gazebo import Gazebo

@neuros_initialise()
def initialise(node):
    node.get_ros_node().get_logger().info("Launching Gazebo")
    simulator = Gazebo(node, os.environ["NEUROS_MODEL"])
    simulator.step()
    node.set_user_data(simulator)

@neuros_function(inputs="request", outputs="_request")
def handle_request_new_level(node, request):
    node.get_ros_node().get_logger().info(f"Going to level: {request.data}")
    return request

@neuros_function(inputs=Optional("_level"), outputs=Optional("level"))
def step(node, level):
    simulator = node.get_user_data()
    simulator.step(1000)
    if level is not None:
        logger = node.get_ros_node().get_logger()
        logger.info(f"Currently at level: {level.data}")
        return level
