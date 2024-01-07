# Copyright (c) 2023 Lee Perry

import os

import tensorflow.compat.v1 as tf

from neuros.hooks import neuros_initialise, neuros_function

class Model:

    def __init__(self):
        pass
        #saver = tf.train.Saver()
        #config = tf.ConfigProto(device_count={'GPU' : 1})
        #sess = tf.Session(config=config)
        #saver.restore(sess, os.environ["NEUROS_CHECKPOINT"])

@neuros_initialise()
def initialise(node):
    model = Model()
    node.set_user_data(model)

@neuros_function(inputs="camera")
def receive_camera_image(node, image):
    node.get_ros_node().get_logger().info("PredNet received camera image")
