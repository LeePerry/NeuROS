# Copyright (c) 2023 Lee Perry

from neuros.hooks import neuros_initialise, neuros_function

import collections
import math
import matplotlib.pyplot as plt
import sys

def timestamp_to_seconds(ts):
    return ts.sec + (ts.nanosec / 1_000_000_000)

class Visualiser:

    class PlottedPose:

        def __init__(self, ax, label, colour):
            self.ax = ax
            self.label = label
            self.colour = colour
            max_length = 5 * 60
            self.x_values = collections.deque(maxlen=max_length)
            self.y_values = collections.deque(maxlen=max_length)
            self.line = None

        def plot(self, message):
            if self.line:
                self.line.remove()
            seconds = timestamp_to_seconds(message.header.stamp)
            self.x_values.append(seconds)
            rot = message.pose.orientation
            yaw = math.atan2(2.0 * (rot.w * rot.z + rot.x * rot.y),
                             1.0 - 2.0 * (rot.y * rot.y + rot.z * rot.z))
            self.y_values.append(yaw)
            self.ax.set_xlim(self.x_values[-1] - 60, self.x_values[-1])
            self.line, = self.ax.plot(
                self.x_values, self.y_values, c=self.colour, label=self.label)

    def __init__(self):
        plt.rcParams['figure.figsize'] = [10, 2]
        plt.rcParams["figure.subplot.left"] = 0.065
        plt.rcParams["figure.subplot.right"] = 0.975
        plt.rcParams["figure.subplot.bottom"] = 0.25
        plt.rcParams["figure.subplot.top"] = 0.84
        self.fig, self.ax = plt.subplots()
        self.ax.set_title('Head Direction')
        self.ax.set_xlabel('Time (seconds)')
        self.ax.set_ylabel('Angle (radians)')
        self.ax.set_yticks([-math.pi, 0, math.pi])
        self.ax.set_yticklabels(["$-\pi$", "$0$", "$\pi$"])
        self.ax.set_ylim(-math.pi, math.pi)
        self.gt_line = Visualiser.PlottedPose(self.ax, "Ground Truth", "g")
        self.estimate_line = Visualiser.PlottedPose(self.ax, "Estimate", "o")
        plt.ion()
        plt.show()

    def _repaint(self):
        self.fig.legend(loc="upper right")
        plt.pause(sys.float_info.min)

    def plot_ground_truth(self, ground_truth):
        self.gt_line.plot(ground_truth)
        self._repaint()

    def plot_pose_estimate(self, pose_estimate):
        self.estimate_line.plot(pose_estimate)
        self._repaint()

@neuros_initialise()
def initialise(node):
    node.set_user_data(Visualiser())

@neuros_function(inputs="ground_truth")
def receive_ground_truth(node, ground_truth):
    visualiser = node.get_user_data()
    visualiser.plot_ground_truth(ground_truth)

@neuros_function(inputs="pose_estimate")
def receive_ground_truth(node, estimate):
    visualiser = node.get_user_data()
    visualiser.plot_pose_estimate(estimate)
