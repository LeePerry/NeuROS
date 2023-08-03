# Copyright (c) 2023 Lee Perry

import os

from neuros.hooks import neuros_initialise, neuros_tick
from neuros.gazebo import Gazebo

@neuros_initialise()
def initialise_simulation(node):
    node.get_ros_node().get_logger().info("Launching Gazebo")
    simulator = Gazebo(node, os.environ["NEUROS_WORLD"])
    node.set_user_data(simulator)
    simulator.step(1)

@neuros_tick(seconds=1.0)
def step_simulation(node):
    logger = node.get_ros_node().get_logger()
    logger.info("TOPICS:")
    for topic in node.get_gazebo_topic_list():
        #logger.info("================================")
        logger.info(topic)
        #logger.info(node.get_gazebo_topic_info(topic))
        #logger.info("================================")
