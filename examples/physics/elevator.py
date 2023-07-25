# Copyright (c) 2023 Lee Perry

from neuros.hooks import neuros_initialise, neuros_function, Optional
from neuros.gazebo import Gazebo

@neuros_initialise()
def initialise_simulation(node):
    node.get_ros_node().get_logger().info("Launching Gazebo")
    simulator = Gazebo(node)
    node.set_user_data(simulator)
    simulator.step(1)

@neuros_function(inputs="request", outputs="_request")
def forward_request(node, request):
    node.get_ros_node().get_logger().info(f"Going to level: {request.data}")
    return request

@neuros_function(inputs=Optional("_level"), outputs=Optional("level"))
def step_simulation(node, level):
    simulator = node.get_user_data()
    simulator.step(1000)
    if level is not None:
        logger = node.get_ros_node().get_logger()
        logger.info(f"Currently at level: {level.data}")
        return level
