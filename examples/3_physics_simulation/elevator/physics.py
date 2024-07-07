# Copyright (c) 2023 Lee Perry

"""
This example demonstrates the creation, control and inspection of a physics
simulation in Gazebo. Specifically, an elevator is commanded to reach a random
level, upon which another random level is selected.
"""

import os

from neuros.hooks import neuros_initialise, neuros_function, Optional
from neuros.gazebo import Gazebo

class Physics:

    def __init__(self, node):
        self.simulator = Gazebo(node, os.environ["NEUROS_MODEL"])
        self.timestamp = 0
        self.async_execution = (os.environ.get("NEUROS_ASYNC_SIM") == "enabled")
        if self.async_execution:
            self.simulator.run_asynchronous()

    def step(self):
        if not self.async_execution:
            self.simulator.step_synchronous()

    def should_log_timestamp(self, seconds):
        if seconds > self.timestamp:
            self.timestamp = seconds + 1
            return True
        return False

@neuros_initialise()
def initialise(node):
    """
    The initialisation hook creates the simulation using the model configured
    via environment variable injection. The simulation is stored inside the
    node's user data for access later on.
    """
    node.set_user_data(Physics(node))

@neuros_function(inputs="command", outputs="_command")
def handle_command(node, command):
    """
    This hook is invoked when a new level command is received from another note.
    This node simply forwards the packet on to Gazebo via an external_topic.
    """
    node.get_ros_node().get_logger().info(f"Going to level: {command.data}")
    return command

@neuros_function(inputs="_clock")
def log_sim_time(node, clock):
    simulation = node.get_user_data()
    seconds = clock.clock.sec + (clock.clock.nanosec / 1_000_000_000)
    if simulation.should_log_timestamp(seconds):
        node.get_ros_node().get_logger().info(f"Simulated time: {seconds}")

@neuros_function(inputs=Optional("_sensor_data"),
                 outputs=Optional("sensor_data"))
def step(node, sensor_data):
    """
    Since this hook accepts only Optional inputs, it will be invoked repeatedly
    until the discard_limit for the destination node is reached. This
    represents the main simulation loop and is kept as tight as possible in
    order to saturate the host machine CPU/GPU.
    """
    simulation = node.get_user_data()
    simulation.step()
    if sensor_data is not None:
        node.get_ros_node().get_logger().info(f"Level: {sensor_data.data}")
        return sensor_data
