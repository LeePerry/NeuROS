# Copyright (c) 2023 Lee Perry

import time

from neuros.hooks import neuros_initialise
from neuros.gazebo import Gazebo

@neuros_initialise()
def set_up_simulator(node):
    sim = Gazebo(node.get_neuros_parameter("world"))
    #time.sleep(5)
    sim.list_topics()
    sim.load_model(node.get_neuros_parameter("model"))
    sim.list_topics()
