# Copyright (c) 2024 Lee Perry

import numpy as np

from neuros.hooks import neuros_initialise, neuros_function, Optional

from model import Model

@neuros_initialise()
def initialise(node):
    node.set_user_data(Model())

@neuros_function(inputs=["imu", Optional("odom_correction")],
                 outputs="odom_estimate")
def estimate_odometry(node, imu, odom_correction):
    model = node.get_user_data()
    if odom_correction:
        model.apply_correction(np.array(odom_correction.data))
    odom = model.estimate(imu)
    odom_estimate = node.make_packet("odom_estimate")
    odom_estimate.data = int(np.argmax(odom))
    return odom_estimate
