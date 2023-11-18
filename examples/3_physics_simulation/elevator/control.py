# Copyright (c) 2023 Lee Perry

"""
This example demonstrates the creation, control and inspection of a physics
simulation in Gazebo. Specifically, an elevator is commanded to reach a random
level, upon which another random level is selected.
"""

import random

from neuros.hooks import neuros_initialise, neuros_function, Optional

def random_command(node, exclude):
    """
    This method generates a random level which is not equal to the current
    level.
    """
    command = node.make_packet("command")
    valid_commands = [0, 1, 2, 3]
    valid_commands.remove(exclude)
    command.data = random.choice(valid_commands)
    node.get_ros_node().get_logger().info(f"Commanding level {command.data}")
    node.set_user_data(command.data)
    return command

@neuros_initialise(outputs="command")
def initialise_controller(node):
    """
    The initialisation hook sends the first random level command to the
    elevator simulation.
    """
    return random_command(node, 0)

@neuros_function(inputs="sensor_data", outputs=Optional("command"))
def handle_sensor_data(node, sensor_data):
    """
    Upon receipt of new sensor data (the current level) this controller checks
    to see if we have arrived at the command level. If so, it responds with a
    new random level command.
    """
    if sensor_data.data == node.get_user_data():
        return random_command(node, sensor_data.data)
