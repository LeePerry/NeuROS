# Copyright (c) 2023 Lee Perry

import os

from neuros.hooks import neuros_initialise, neuros_function, Optional
from neuros.gazebo import Gazebo

@neuros_initialise()
def initialise(node):
    simulator = Gazebo(node, os.environ["NEUROS_MODEL"])
    simulator.step()
    node.set_user_data(simulator)

@neuros_function(inputs="command", outputs="_command")
def handle_command(node, command):
    node.get_ros_node().get_logger().info(f"Going to level: {command.data}")
    return command

@neuros_function(inputs=Optional("_sensor_data"),
                 outputs=Optional("sensor_data"))
def step(node, sensor_data):
    simulator = node.get_user_data()
    simulator.step()
    if sensor_data is not None:
        logger = node.get_ros_node().get_logger()
        logger.info(f"Currently at level: {sensor_data.data}")
        return sensor_data
