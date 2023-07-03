# Copyright (c) 2023 Lee Perry

from neuros.hooks import neuros_initialise, neuros_tick
from neuros.gazebo import Gazebo

@neuros_initialise()
def set_up_simulator(node):
    node.set_user_data(Gazebo(node))

@neuros_tick(seconds=1.0)
def step_simulator(node):

    logger = node.get_ros_node().get_logger()
    logger.info("Tick")

    simulator = node.get_user_data()
    simulator.step(1)

    #entity = simulator.get_entity_by_name("quadcopter")
    #logger.info(f"quadcopter is entity {entity}")

    #logger.info("======== ROS TOPICS ========")
    #for topic in node.get_ros_topic_list():
    #    logger.info(topic)

    logger.info("======== IGN TOPICS ========")
    for topic in node.get_ign_topic_list():
        logger.info(topic)
        logger.info(node.get_ign_topic_message_type(topic))
