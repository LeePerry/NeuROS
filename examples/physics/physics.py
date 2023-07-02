# Copyright (c) 2023 Lee Perry

from neuros.hooks import neuros_initialise
from neuros.gazebo import Gazebo

@neuros_initialise()
def set_up_simulator(node):
    node.set_user_data(Gazebo(node.get_parameter("world")))

    for topic in node.get_topic_list():
        print(topic)
