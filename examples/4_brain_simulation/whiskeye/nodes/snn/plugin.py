# Copyright (c) 2023 Lee Perry

import nest

from neuros.hooks import neuros_initialise, neuros_function

@neuros_initialise()
def initialise(node):
    pass

@neuros_function(inputs="imu")
def receive_camera_image(node, image):
    node.get_ros_node().get_logger().info("SNN received IMU data")
