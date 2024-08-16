# Copyright (c) 2024 Lee Perry

import numpy as np

from neuros.hooks import neuros_initialise, neuros_function, Optional

from model import Model

def timestamp_to_seconds(ts):
    return ts.sec + (ts.nanosec / 1_000_000_000)

@neuros_initialise()
def initialise(node):
    node.set_user_data(Model(ring_model='Grid'))

@neuros_function(inputs=["imu", Optional("odom_correction")],
                 outputs=Optional("odom_estimate"))
def estimate_odometry(node, imu, odom_correction):
    model = node.get_user_data()
    if odom_correction:
        model.apply_correction(np.array(odom_correction.data),
                               imu.header.stamp)
    estimate = model.estimate(imu)
    if estimate is not None:

        angle = ((int(estimate) % 120) * (360 // 120) - 180)
        t = timestamp_to_seconds(imu.header.stamp)
        node.get_ros_node().get_logger().info(
            f"Spiking Neural Network: [{angle}, {t}]")

        odom_estimate = node.make_packet("odom_estimate")
        odom_estimate.data = int(estimate)
        return odom_estimate
