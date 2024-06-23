# Copyright (c) 2023 Lee Perry

import math
import os
import random

from neuros.hooks import neuros_initialise, neuros_function, Optional
from neuros.gazebo import Gazebo

def timestamp_to_seconds(ts):
    return ts.sec + (ts.nanosec / 1_000_000_000)

class Robot:

    STEP_SIZE       = 1
    SENSORS         = ["ground_truth", "camera", "imu"]
    NECK_COMMANDS   = ["_body_neck_pos", "_neck_gmbl_pos", "_gmbl_head_pos"]
    MOTION_COMMANDS = ["_pose_x_vel", "_pose_y_vel", "_pose_theta_vel"]

    def __init__(self, node):
        self.node = node
        self.simulator = Gazebo(node, os.environ["NEUROS_WORLD"])
        self.simulator.spawn_entity(os.environ["NEUROS_ROBOT"])
        self.realtime = int(os.environ["NEUROS_REALTIME"])
        self.randomise = int(os.environ["NEUROS_RANDOMISE_DIRECTION"])
        self.first_motion = True
        self.motion_timeout = 0
        self.motion_direction = 1.0
        self.first_step = True
        
    def initialise_neck(self):
        body_neck = self.node.make_packet("_body_neck_pos")
        body_neck.data = 0.0
        neck_gmbl = self.node.make_packet("_neck_gmbl_pos")
        neck_gmbl.data = math.pi / 2
        gmbl_head = self.node.make_packet("_gmbl_head_pos")
        gmbl_head.data = 0.0
        return (body_neck, neck_gmbl, gmbl_head)

    def perform_motion(self, ground_truth):
        if (self.first_motion or self.randomise) and ground_truth:
            seconds = timestamp_to_seconds(ground_truth.header.stamp)
            if seconds > self.motion_timeout:
                self.first_motion = False
                pose_x = self.node.make_packet("_pose_x_vel")
                pose_x.data = 0.0
                pose_y = self.node.make_packet("_pose_y_vel")
                pose_y.data = 0.0
                pose_theta = self.node.make_packet("_pose_theta_vel")
                pose_theta.data = (
                    float(os.environ["NEUROS_ROTATION_SPEED"]) *
                    self.motion_direction)
                self.motion_timeout = seconds + random.randint(3, 20)
                self.motion_direction *= -1
                return (pose_x, pose_y, pose_theta)
        return (None, None, None)

    def step(self):
        if self.realtime:
            if self.first_step:
                self.first_step = False
                self.simulator.run_asynchronous()
        else:
            self.simulator.step_synchronous(Robot.STEP_SIZE)

@neuros_initialise(outputs = Robot.NECK_COMMANDS)
def initialise(node):
    robot = Robot(node)
    node.set_user_data(robot)
    return robot.initialise_neck()

@neuros_function(
    inputs  = [Optional(f"_{i}") for i in Robot.SENSORS],
    outputs = [Optional(i) for i in Robot.SENSORS + Robot.MOTION_COMMANDS])
def step(node, *sensor_data):
    robot = node.get_user_data()
    robot.step()
    if robot.simulator.get_sim_time().total_seconds() > 3:
        return sensor_data + robot.perform_motion(sensor_data[0])
