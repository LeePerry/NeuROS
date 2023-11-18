# Copyright (c) 2023 Lee Perry

"""
This example demonstrates the creation, control and inspection of a physics
simulation in Gazebo. Specifically, an elevator is commanded to reach a random
level, upon which another random level is selected.
"""

import os

from neuros.hooks import neuros_initialise, neuros_function, Optional
from neuros.gazebo import Gazebo

@neuros_initialise()
def initialise(node):
    """
    The initialisation hook creates the simulation using the model configured
    via environment variable injection. The simulation is stored inside the
    node's user data for access later on.
    """
    node.set_user_data(Gazebo(node, os.environ["NEUROS_MODEL"]))

@neuros_function(inputs="command", outputs="_command")
def handle_command(node, command):
    """
    The hook is invoked when a new level command is received from another note.
    This node simply forwards the packet on to Gazebo via an external_topic.
    """
    node.get_ros_node().get_logger().info(f"Going to level: {command.data}")
    return command

@neuros_function(inputs=Optional("_sensor_data"),
                 outputs=Optional("sensor_data"))
def step(node, sensor_data):
    """
    Since this hook accepts only Optional inputs, it will be invoked repeatedly
    until the discard_limit for the destination node is reached. This
    represents the main simulation loop and is kept as tight as possible in
    order to saturate the host machine CPU/GPU.
    """
    simulator = node.get_user_data()
    simulator.step()
    if sensor_data is not None:
        logger = node.get_ros_node().get_logger()
        logger.info(f"Currently at level: {sensor_data.data}")
        return sensor_data
