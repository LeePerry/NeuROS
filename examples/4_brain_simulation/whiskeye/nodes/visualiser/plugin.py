# Copyright (c) 2024 Lee Perry

import numpy as np
import time

from neuros.hooks import neuros_initialise, neuros_function
from neuros.visualise import VisualiseRotations

GROUND_TRUTH    = "Ground Truth"
PRED_NET_OUTPUT = "Predictive Coding Network"
SNN_OUTPUT      = "Spiking Neural Network"

@neuros_initialise()
def initialise(node):
    visualiser = VisualiseRotations('Head Direction', duration=60*5)
    visualiser.add_quaternion(GROUND_TRUTH, "g")
    visualiser.add_degrees(PRED_NET_OUTPUT, "b")
    visualiser.add_degrees(SNN_OUTPUT, "r")
    node.set_user_data((time.time(), visualiser))

@neuros_function(inputs="ground_truth")
def receive_ground_truth(node, ground_truth):
    started, visualiser = node.get_user_data()
    visualiser.plot(GROUND_TRUTH,
                    ground_truth.pose.orientation,
                    (time.time() - started) / 10.0)

@neuros_function(inputs="head_dir_prediction")
def receive_head_dir_prediction(node, prediction):
    started, visualiser = node.get_user_data()
    visualiser.plot(PRED_NET_OUTPUT,
                    (np.argmax(prediction.data) * 2) - 180,
                    (time.time() - started) / 10.0)

@neuros_function(inputs="odom_estimate")
def receive_odom_estimate(node, estimate):
    started, visualiser = node.get_user_data()
    
    angle = ((estimate.data % 120) * (360 // 120) - 180)

    node.get_ros_node().get_logger().info(f"odom_estimate = {estimate.data} [{angle}]")
    visualiser.plot(SNN_OUTPUT,
                    angle,
                    (time.time() - started) / 10.0)
