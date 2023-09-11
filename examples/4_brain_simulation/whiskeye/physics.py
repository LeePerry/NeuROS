# Copyright (c) 2023 Lee Perry

import os

from neuros.hooks import neuros_initialise, neuros_tick
from neuros.gazebo import Gazebo

@neuros_initialise()
def initialise(node):
    simulator = Gazebo(node, os.environ["NEUROS_WORLD"])
    simulator.spawn_entity(os.environ["NEUROS_ROBOT"])
    simulator.step(1)
    node.set_user_data(simulator)

@neuros_tick(seconds=1.0)
def step(node):
    simulator = node.get_user_data()
    simulator.step()

    print("======== Topics...")
    for topic in simulator.get_topic_list():
        print(topic)
    print("======== Services...")
    for topic in simulator.get_service_list():
        print(topic)
    print("======== End.")
