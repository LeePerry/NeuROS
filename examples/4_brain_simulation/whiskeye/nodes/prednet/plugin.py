# Copyright (c) 2024 Lee Perry

import cv2
from cv_bridge import CvBridge
import numpy as np
import os
from scipy.stats import laplace
from scipy.signal import resample

from neuros.hooks import neuros_initialise, neuros_function
from neuros.config import FileSystem

from model.model import Model

@neuros_initialise()
def initialise(node):
    model = Model(node.get_ros_node().get_logger(),
                  os.path.join(FileSystem.standard_project_dir,
                               "nodes/prednet/model"))
    node.set_user_data(model)

@neuros_function(inputs=["camera", "odom_estimate"], # TODO Accumulate(odom_estimate, 5) ?
                 outputs="head_dir_prediction")
def predict_head_direction(node, image, odom_estimate):

    # prepare image
    image = CvBridge().imgmsg_to_cv2(image, "rgb8") # bgr8
    image = cv2.resize(image, (80, 45))
    image = image.reshape(1, 10800)

    # prepare odometry distribution
    laplacian_range = np.arange(-(120 // 2), (120 // 2))
    zeroed_laplacian = laplace(0, 120 // 20).pdf(laplacian_range)
    bump_centre = odom_estimate.data
    shifted = np.roll(zeroed_laplacian, bump_centre - (120 // 2))
    scaling_factor = 1 / shifted.max()
    shifted = shifted * scaling_factor
    shifted = resample(x=shifted, num=180, axis=0)
    shifted = shifted.reshape(1, 180)

    # make inference
    model = node.get_user_data()
    _, _, direction = model.predict(visual_data=image, odometry_data=shifted)
    head_dir_prediction = node.make_packet("head_dir_prediction")
    head_dir_prediction.data = [float(i) for i in direction[0]]
    return head_dir_prediction
