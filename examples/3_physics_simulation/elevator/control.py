# Copyright (c) 2023 Lee Perry

import random

from neuros.hooks import neuros_initialise, neuros_function, Optional

def random_command(node, exclude):
    command = node.make_packet("command")
    valid_commands = [0, 1, 2, 3]
    valid_commands.remove(exclude)
    command.data = random.choice(valid_commands)
    node.get_ros_node().get_logger().info(f"Commanding level {command.data}")
    node.set_user_data(command.data)
    return command

@neuros_initialise(outputs="command")
def initialise_controller(node):
    return random_command(node, 0)

@neuros_function(inputs="sensor_data", outputs=Optional("command"))
def handle_sensor_data(node, sensor_data):
    if sensor_data.data == node.get_user_data():
        return random_command(node, sensor_data.data)
