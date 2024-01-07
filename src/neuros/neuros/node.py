#!/usr/bin/env python3
# Copyright (c) 2023 Lee Perry

"""
This module represents an individual NeuROS node.
"""

import importlib.util
import os
import subprocess

import geometry_msgs.msg
import rclpy
import rclpy.node
import std_msgs.msg
import sensor_msgs.msg

from neuros.hooks import Hooks
from neuros.config import NodeConfig, FileSystem

class Node:

    """
    The entry-point and main application logic for all NeuROS nodes.

    This class is passed to user defined callbacks.

    This module is also the only NeuROS module which should include any
    external dependencies. All other modules will invoke methods on this
    class in order to execute 3rd-party functionality. The interface of
    this node therefor provides the interface for dependency injection
    into the rest of the system.
    """

    @classmethod
    def _make_qos(cls, reliable):
        """
        Constructs and returns a new Quality of Service profile for publishers
        and subscribers.

        Parameters:
            reliable (boolean) : True indicates that the QoS should use TCP.
                                 False indicates that the QoS should use UDP.
        """
        return rclpy.qos.QoSProfile(
            reliability = rclpy.qos.ReliabilityPolicy.RELIABLE
                          if reliable else
                          rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability  = rclpy.qos.DurabilityPolicy.VOLATILE,
            history     = rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth       = 1)

    def __init__(self, config):
        """
        The class constructor, which creates the associated ROS node, injects
        environment variables into the plugin env, and initialises the hooks
        associated provided by the plugin.

        Parameters:
            config (NodeConfig): The configuration settings for this instance.
        """
        self._name = config.name
        self._node = rclpy.create_node(self._name)
        self._config = config
        self._plugins = set()
        self._user_data = None
        for k, v in self._config.env.items():
            os.environ[k] = v

        # this must always be the last line as callbacks are invoked here!
        self._hooks = Hooks(self, config)

    def get_ros_node(self):
        """
        Getter for the underlying ROS node.

        Returns:
            The ROS node used for communication.
        """
        return self._node

    def set_user_data(self, data):
        """
        Setter for arbitrary user data.

        This is not used directly by NeuROS and exists solely so that users
        have a convenient method for associating some additional state with a
        NeuROS node. This can be particularly useful if the same plugin is used
        for multiple nodes.

        For examples of how this can be used, refer to one of the included
        physics simulation example.

        Parameters:
            data: User data top store with this node.
        """
        self._user_data = data

    def get_user_data(self):
        """
        Getter for arbitrary user data.

        Refer to set_user_data for more information.

        Returns:
           Whatever data was passed to set_user_data, or None if it has never
           been called.
        """
        return self._user_data

    def get_topic_list(self):
        """
        Returns the full list of available ROS topics. This is not used
        directly by NeuROS but is included as a useful debugging tool for user
        plugins.

        Returns:
            A list of strings, where each string represents a single topic.
        """
        return [str(topic, "utf-8") for topic in
                subprocess.check_output("ros2 topic list", shell=True).split()]

    def get_topic_info(self, topic):
        """
        Returns a human-readable description of a specific topic, including the
        supported message type. This is not used directly by NeuROS but is
        included as a useful debugging tool for user plugins.

        Parameters:
            topic (str): A valid ROS topic which needs to be inspected and
                         described.

        Returns:
            A string containing a human-readable description of the topic
            details.
        """
        return str(subprocess.check_output(
            f"ros2 topic info {topic}", shell=True), "utf-8")

    def load_plugin(self, directory, filename):
        """
        Loads a user specifided node plugin (containing Python code).

        Parameters:
            directory (str): The absolute path to th directory containing the
                             plugin.
            filename (str): The filename (including .py extension) of the
                            plugin.
        """
        module_name = os.path.splitext(filename)[0]
        full_path = os.path.join(directory, filename)
        if full_path not in self._plugins:
            spec = importlib.util.spec_from_file_location(module_name, full_path)
            impl = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(impl)
        self._plugins.add(full_path)

    def find_type_by_name(self, name):
        """
        Attempts to find and return the Python type identified by the specified
        name. If the type cannot be found then an excpetion will be raised.

        Parameters:
            name (str): The name of the type to be found, as would be valid in
                        Python code. This includes support for any ROS2 message
                        types (such as those found in std_msgs.msg,
                        sensor_msgs.msg and geometry_msgs.msg), in addition to
                        custom message types imported into plugins by users.

        Returns:
            The Python type corresponding to the specified name.
        """
        try:
            obj = eval(name)
            if isinstance(obj, type):
                return obj
        except Exception:
            pass
        raise Exception(f"Invalid plugin type: {name}") from None

    def make_packet(self, output_name=None):
        """
        A factory method for the packet type associated with a named output
        (or std_msgs.msg.Empty by default).

        Parameters:
            output_name (str) : The name of an outpt associated with this node.

        Returns:
            A new packet of the type associated with the named output.
        """
        return (std_msgs.msg.Empty()
                if output_name is None else
                self._hooks.outputs[output_name].create_packet())

    def make_loop_back(self, callback):
        """
        A factory for a publisher / subscriber pair that can be used as a
        loop-back message queue for this node. This is useful for executing an
        action after all existing queued messages have been processed.

        Parameters:
            callback (func): A callback method to be called when the subscriber
                             receives a new packet.

        Returns:
            A tuple containing one subscriber and one publisher in that order.
        """
        topic = "/".join(["neuros", self._name, "loop"])
        return (self._node.create_subscription(
                    std_msgs.msg.Empty,
                    topic,
                    callback,
                    Node._make_qos(True)),
                self._node.create_publisher(
                    std_msgs.msg.Empty,
                    topic,
                    Node._make_qos(True)))

    def make_external_input(self, topic, packet_type, callback):
        """
        A factory for a single subscriber, which can be used for subscribing
        to topics outside of NeuROS. Note that when doing this all guarantees
        provided by NeuROS are abandoned and functionality is reduced to raw
        ROS2 based communication.

        Parameters:
            topic (str): A string representation of the topic to subscribe to.
            packet_type (type): The Python type of the packet associated with
                                the specified topic.
            callback (func): A callback method to be called when the subscriber
                             receives a new packet.

        Returns:
            A single subscriber.
        """
        return self._node.create_subscription(
            packet_type, topic, callback, Node._make_qos(True))

    def make_internal_input(self, connection, packet_type, callback):
        """
        A factory for a NeuROS based input, comprising of a subscriber for
        receiving data and two publishers for registering and acknowledging
        receipt.

        Parameters:
            connection (ConnectionConfig): The associated connection config.
            packet_type (type): The Python type associated with this input.
            callback (func): A callback method to be called when the subscriber
                             receives a new packet.

        Returns:
            A tuple containing (in the following order) a subscriber for
            receiving data, a publisher for sending receipt acknowledgements
            and finally a publisher for registering this end of the connection.
        """
        topic = "/".join(["neuros",
                          connection.source_node,
                          connection.source_output,
                          connection.destination_node])
        return (self._node.create_subscription(
                    packet_type,
                    f"{topic}/data",
                    callback,
                    Node._make_qos(connection.is_reliable)),
                self._node.create_publisher(
                    std_msgs.msg.Empty,
                    f"{topic}/ack",
                    Node._make_qos(True))
                    if connection.is_reliable else None,
                self._node.create_publisher(
                    std_msgs.msg.Empty,
                    f"{topic}/reg",
                    Node._make_qos(True)))

    def make_external_output(self, topic, packet_type):
        """
        A factory for a single publisher, which can be used for publishing
        to topics outside of NeuROS. Note that when doing this all guarantees
        provided by NeuROS are abandoned and functionality is reduced to raw
        ROS2 based communication.

        Parameters:
            topic (str): A string representation of the topic to publish to.
            packet_type (type): The Python type of the packet associated with
                                the specified topic.

        Returns:
            A single publisher.
        """
        return self._node.create_publisher(
            packet_type, topic, Node._make_qos(True))

    def make_internal_output(self, connection, packet_type, ack_cb, reg_cb):
        """
        A factory for a NeuROS based output, comprising of a publisher for
        receiving data and two subscribers for registration and acknowlements.

        Parameters:
            connection (ConnectionConfig): The associated connection config.
            packet_type (type): The Python type associated with this input.
            ack_cb (func): A callback method to be called when a recipient
                           acknowledges a packet has been recived.
            reg_cb (func): A callback method to be called when a recipient
                           registers as ready to receieve this data.

        Returns:
            A tuple containing (in the following order) a publisher for
            sending data, a subscriber for receiving receipt acknowledgements
            and finally a subscriber for receiving registrations.
        """
        topic = "/".join(["neuros",
                          connection.source_node,
                          connection.source_output,
                          connection.destination_node])
        return (self._node.create_publisher(
                    packet_type,
                    f"{topic}/data",
                    Node._make_qos(connection.is_reliable)),
                self._node.create_subscription(
                    std_msgs.msg.Empty,
                    f"{topic}/ack",
                    ack_cb,
                    Node._make_qos(True)),
                self._node.create_subscription(
                    std_msgs.msg.Empty,
                    f"{topic}/reg",
                    reg_cb,
                    Node._make_qos(True)))

    def make_timer(self, seconds, callback):
        """
        A factory for a timer.

        Parameters:
            seconds (double): The period the timer should wait between
                              invocations of callback.
            callback (func): A callback method to be called wheneevr the timer
                             expires.

        Returns:
            A single timer object.
        """
        return self._node.create_timer(seconds, callback)

def main():
    """
    The main entry point for a NeuROS node.

    This method loads the node config from the standardised node directory,
    creates a corresponding NeuROS node and spins it with a single-threaded
    executor.

    This method will block indefinitely, until the user interrupts with Ctrl+C.
    """
    config = NodeConfig.from_standard_node_dir()
    rclpy.init()
    node = Node(config)
    try: rclpy.spin(node.get_ros_node())
    except KeyboardInterrupt: pass
    rclpy.try_shutdown()

if __name__ == '__main__':
    main()
