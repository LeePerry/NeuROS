# Copyright (c) 2024 Lee Perry

from neuros.hooks import neuros_initialise, neuros_function
from neuros.visualise import VisualiseRotations

GROUND_TRUTH = "Ground Truth"
ESTIMATE = "Estimate"

def timestamp_to_seconds(ts):
    return ts.sec + (ts.nanosec / 1_000_000_000)

@neuros_initialise()
def initialise(node):
    visualiser = VisualiseRotations('Head Direction')
    visualiser.add_quaternion(GROUND_TRUTH, "g")
    visualiser.add_quaternion(ESTIMATE, "o")
    node.set_user_data(visualiser)

@neuros_function(inputs="ground_truth")
def receive_ground_truth(node, ground_truth):
    visualiser = node.get_user_data()
    visualiser.plot(GROUND_TRUTH,
                    ground_truth.pose.orientation,
                    timestamp_to_seconds(ground_truth.header.stamp))

@neuros_function(inputs="pose_estimate")
def receive_ground_truth(node, estimate):
    visualiser = node.get_user_data()
    visualiser.plot(ESTIMATE,
                    estimate.pose.orientation,
                    timestamp_to_seconds(estimate.header.stamp))
