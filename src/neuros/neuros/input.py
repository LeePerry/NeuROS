# Copyright (c) 2023 Lee Perry

"""
This module provides the communication resources needed for NeuROS nodes
to receive inputs, both from internal and external sources.
"""

from neuros.config import FileSystem

class _ExternalSubscriber:

    """
    A subscription client for data from outside of NeuROS.

    Note that in this case the source_node will be reported as:
        __unknown_external_source__
    """

    def __init__(self, node, input, packet_type, input_cb):
        """
        Initialises an instance of this class.

        Parameters:
            node (Node): The node associated with this input.
            input (Input): The InputConfig for the associated input.
            packet_type: The expected packet type to be received.
            input_cb (function): A callback method to be invoked when new data
                                 is received.
        """
        self.source_node = "__unknown_external_source__"
        self._input_cb = input_cb
        self._data = node.make_external_input(
            input.external_topic, packet_type, self._input_callback)

    def _input_callback(self, packet):
        """
        The callback method which is invoked upon receipt of new data.

        Parameters:
            packet: The newly received data.
        """
        self._input_cb(self.source_node, packet)

class _InternalSubscriber:

    """
    A subscription client for data coming from within NeuROS.

    This additionally provides the client side registration and
    acknowledgement mechanisms on a per-input basis.
    """

    def __init__(self, node, connection, packet_type, input_cb):
        """
        Initialises an instance of this class.

        Parameters:
            node (Node): The node associated with this input.
            connection (ConnectionConfig): The connection configuration
                                           associated with this input.
            packet_type: The expected packet type to be received.
            input_cb (function): A callback method to be invoked when new data
                                 is received.
        """
        self.source_node = connection.source_node
        self._input_cb = input_cb
        self._data, self._ack, self._reg = node.make_internal_input(
            connection, packet_type, self._input_callback)
        self._reg_timer = node.make_timer(0.5, self._send_reg)
        self._packet = node.make_packet()

    def _send_reg(self):
        """
        Send a registration packet to the source_node of the associated input.
        """
        self._reg.publish(self._packet)

    def _input_callback(self, packet):
        """
        The callback method which is invoked upon receipt of new data.

        Parameters:
            packet: The newly received data.
        """
        self._reg_timer.cancel()
        self._ack.publish(self._packet)
        self._input_cb(self.source_node, packet)

class Input:

    """
    A named input for a specific node.

    Note: This single input may be the destination for multiple connections.
    """

    @classmethod
    def for_node(cls, node, config, input_cb):
        """
        Factory method for all inputs corresponding to a particular node.

        Parameters:
            node (Node): The node associated with this input.
            config (NodeConfig): The node configuration containing all
                                 connections associated with this input.
            input_cb (function): A callback method to be invoked when new data
                                 is received.

        Returns:
            A dictionary mapping each input name to an Input instance.
        """
        return {i.name : cls(node, i, config, input_cb)
                for i in config.inputs}

    def __init__(self, node, input, config, input_cb):
        """
        Initialises an instance of this class.

        Parameters:
            node (Node): The node associated with this input.
            input (InputConfig): The input configuration associated with this
                                 input.
            config (NodeConfig): The node configuration containing all
                                 connections associated with this input.
            input_cb (function): A callback method to be invoked when new data
                                 is received.
        """
        self._name = input.name
        if input.plugin:
            node.load_plugin(FileSystem.standard_project_dir, input.plugin)
        if input.external_topic is None:
            self._subscribers = [_InternalSubscriber(node, c,
                node.find_type_by_name(input.type), self._input_callback)
                for c in config.connections
                if c.destination_node  == config.name and
                   c.destination_input == self._name]
        else:
            self._subscribers = [_ExternalSubscriber(node, input,
                node.find_type_by_name(input.type), self._input_callback)]
        self._input_cb = input_cb

    def _input_callback(self, source_node, packet):
        """
        Invoked when input has received new data.

        Parameters:
            source_node (str): The name of the source node that sent the data.
            packet: The data received.
        """
        self._input_cb(self._name, source_node, packet)

    def all_sources(self):
        """
        Getter for the complete list of source nodes.

        Returns:
           A list of strings, where each string uniquely identifies a
           source node for this input by name.
        """
        return [s.source_node for s in self._subscribers]

class Timer:

    """
    A timer class, used to invoke a hook periodically.
    """

    def __init__(self, node, seconds, callback):
        """
        Initialises an instance of this class.

        Parameters:
            node (Node): The node associated with this timer.
            seconds (double): The length of the interval in seconds.
            callback (function): A hook to be invoked when the timer expires.
        """
        self.is_waiting = True
        self._callback = callback
        self._timer = node.make_timer(seconds, self._expired)

    def _expired(self):
        """
        The timer has expired and the hook will be invoked.
        """
        self.is_waiting = False
        self._callback()

    def clear(self):
        """
        Resets the timer, meaning the hook will not be invoked until the timer
        has expired again.
        """
        self.is_waiting = True
