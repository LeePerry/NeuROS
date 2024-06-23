# Copyright (c) 2024 Lee Perry

import numpy as np
import time

from neuros.hooks import neuros_initialise, neuros_function
from neuros.visualise import VisualiseRotations

GROUND_TRUTH = "Ground Truth"
PREDICTION = "Prediction"

def timestamp_to_seconds(ts):
    return ts.sec + (ts.nanosec / 1_000_000_000)

@neuros_initialise()
def initialise(node):
    visualiser = VisualiseRotations('Head Direction', duration=60*5)
    visualiser.add_quaternion(GROUND_TRUTH, "g")
    visualiser.add_degrees(PREDICTION, "b")
    node.set_user_data((time.time(), visualiser))

@neuros_function(inputs="ground_truth")
def receive_ground_truth(node, ground_truth):
    started, visualiser = node.get_user_data()
    visualiser.plot(GROUND_TRUTH,
                    ground_truth.pose.orientation,
                    time.time() - started)
                    #timestamp_to_seconds(ground_truth.header.stamp))

@neuros_function(inputs="head_dir_prediction")
def receive_head_dir_prediction(node, prediction):
    started, visualiser = node.get_user_data()
    visualiser.plot(PREDICTION,
                    np.argmax(prediction.data) * 2,
                    time.time() - started)
                    #timestamp_to_seconds(prediction.header.stamp))
