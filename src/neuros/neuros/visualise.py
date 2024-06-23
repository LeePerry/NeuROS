# Copyright (c) 2024 Lee Perry

"""
This module is responsible for providing real-time graph based visualisations.
"""

import collections
import matplotlib.pyplot as plt
import math
import sys

class _PlotableLine:

    """
    The base class for all data sources that can be plot as a single line.
    """

    def plot(self):
        """
        (Re)plot the line, representing the previously added datapoints.
        """
        if len(self.x_values) == 0:
            return
        if self.line:
            self.line.remove()
        self.ax.set_xlim(self.x_values[-1] - self._duration, self.x_values[-1])
        self.line, = self.ax.plot(
            self.x_values, self.y_values, c=self.colour, label=self.label)

class _Quaternion(_PlotableLine):

    """
    A representation of a quaternion data source for visualisation.
    """

    def __init__(self, duration, ax, label, colour):
        """
        Initialises an instance of this class.

        Parameters:
            duration (int): The duration of time to visualise in seconds.
            ax: The matplotlib axis.
            label (str): The name of the data source.
            colour (char): The colour to render the line as a matplotlib
                           colour code.
        """
        self._duration = duration
        self.ax = ax
        self.label = label
        self.colour = colour
        max_length = 5 * duration
        self.x_values = collections.deque(maxlen=max_length)
        self.y_values = collections.deque(maxlen=max_length)
        self.line = None

    def add(self, orientation, timestamp):
        """
        Add a new datapoint.

        Parameters:
            orientation: The orientation represented as a quaternion.
            timestamp (float): The timestamp in seconds.
        """
        self.x_values.append(timestamp)
        o = orientation
        yaw = math.atan2(2.0 * (o.w * o.z + o.x * o.y),
                         1.0 - 2.0 * (o.y * o.y + o.z * o.z))
        self.y_values.append(yaw)

class _Degrees(_PlotableLine):

    """
    A representation of an angle in degrees data source for visualisation.
    """

    def __init__(self, duration, ax, label, colour):
        """
        Initialises an instance of this class.

        Parameters:
            duration (int): The duration of time to visualise in seconds.
            ax: The matplotlib axis.
            label (str): The name of the data source.
            colour (char): The colour to render the line as a matplotlib
                           colour code.
        """
        self._duration = duration
        self.ax = ax
        self.label = label
        self.colour = colour
        max_length = 5 * duration
        self.x_values = collections.deque(maxlen=max_length)
        self.y_values = collections.deque(maxlen=max_length)
        self.line = None

    def add(self, degrees, timestamp):
        """
        Add a new datapoint.

        Parameters:
            degrees: The angle represented in degrees.
            timestamp (float): The timestamp in seconds.
        """
        self.x_values.append(timestamp)
        self.y_values.append(degrees * math.pi / 180)

class VisualiseRotations:

    """
    A class for plotting rotations against time (in seconds).
    """

    def __init__(self, title, duration=60):
        """
        Initialises an instance of this class.

        Parameters:
            title: The name of this visualisation to display.
            duration (int): The duration of time to visualise in seconds.
        """
        self._duration = duration
        plt.rcParams['figure.figsize'] = [10, 2]
        plt.rcParams["figure.subplot.left"] = 0.065
        plt.rcParams["figure.subplot.right"] = 0.975
        plt.rcParams["figure.subplot.bottom"] = 0.25
        plt.rcParams["figure.subplot.top"] = 0.84
        self._fig, self._ax = plt.subplots(num=title)
        self._ax.set_xlabel('Time (seconds)')
        self._ax.set_ylabel('Angle (radians)')
        self._ax.set_yticks([-math.pi, 0, math.pi])
        self._ax.set_yticklabels(["$-\pi$", "$0$", "$\pi$"])
        self._ax.set_ylim(-math.pi, math.pi)
        self._datasources = {}
        plt.ion()
        plt.show()

    def _repaint(self):
        """
        An internal method for repainting the figure after a change has
        been made.
        """
        self._fig.legend(loc="upper right")
        plt.pause(sys.float_info.min)

    def add_quaternion(self, name, colour):
        """
        Add a new quaternion datasource.

        Parameters:
            name (str): The name of the datasource.
            colour (char): The colour of the datasource as a matplotlib
                           colour code.
        """
        self._datasources[name] = _Quaternion(
            self._duration, self._ax, name, colour)

    def add_degrees(self, name, colour):
        """
        Add a new angle in degrees datasource.

        Parameters:
            name (str): The name of the datasource.
            colour (char): The colour of the datasource as a matplotlib
                           colour code.
        """
        self._datasources[name] = _Degrees(
            self._duration, self._ax, name, colour)

    def plot(self, line_name, orientation, timestamp):
        """
        Plot a new datapoint.

        Parameters:
            orientation: The orientation, in a suitable representation.
            timestamp (float): The timestamp in seconds.
        """
        self._datasources[line_name].add(orientation, timestamp)
        for _, datasource in self._datasources.items():
            # always plot all data sources in the same order, in order to 
            # retain key order
            datasource.plot()
        self._repaint()
