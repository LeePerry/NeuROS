
# Copyright (c) 2023 Lee Perry

"""
This module provides the communication resources needed for NeuROS nodes
to send outputs, both internally and to external destinations.
"""

from neuros.config import FileSystem

class _ExternalPublisher:

    """
    A publisher used to send data outside of NeuROS.
    """

    def __init__(self, node, output, packet_type):
        """
        Initialises an instance of this class.

        Parameters:
            node (Node): The node associated with this output.
            output (OutputConfig): The OutputConfig for the associated output.
            packet_type: The type of the data to be sent.
        """
        self.is_registered = True
        self.is_blocked = False
        self._data_pub = node.make_external_output(
            output.external_topic, packet_type)

    def publish(self, packet):
        """
        Publish a packet.

        Parameters:
            packet: The data to be published.
        """
        self._data_pub.publish(packet)

class _InternalPublisher:

    """
    A publisher for sending data to destinations within NeuROS.

    This additionally provides the server side registration and
    acknowledgement mechanisms on a per-output basis.
    """

    def __init__(self, node, connection, packet_type, reg_cb, ack_cb):
        """
        Initialises an instance of this class.

        Parameters:
            node (Node): The node associated with this output.
            connection (ConnectionConfig): The connection config associated
                                           with this output.
            packet_type: The type of the data to be sent.
            reg_cb (function): A hook to be invoked when a registration is
                               received from a destination node.
            ack_cb (function): A hook to be invoked when an acknowledgement is
                               received from a destination node.
        """
        self._logger = node.get_ros_node().get_logger()
        self.is_registered = False
        self._destination_node = connection.destination_node
        self._source_output = connection.source_output
        self._logger.info(f"Waiting until {self._destination_node} "
            f"is ready to receive {self._source_output}...")
        self.is_blocked = False
        self._is_reliable = connection.is_reliable
        self._discard_limit = connection.discard_limit
        self._acks_pending = 0
        self._reg_cb = reg_cb
        self._ack_cb = ack_cb
        self._data_pub, self._ack_sub, self._reg_sub = node.make_internal_output(
            connection, packet_type, self._ack_callback, self._reg_callback)

    def _reg_callback(self, _):
        """
        A registration was received from a destination node.
        """
        if not self.is_registered:
            self.is_registered = True
            self._logger.info(f"{self._destination_node} "
                f"is now ready to receive {self._source_output}!")
        self._reg_cb()

    def _ack_callback(self, _):
        """
        An acknowledgement was received from a destination node.
        """
        self.is_blocked = False
        self._acks_pending = 0
        self._ack_cb()

    def publish(self, packet):
        """
        Send a packet to the destination node.
        """
        self._data_pub.publish(packet)
        if self._is_reliable:
            self._acks_pending += 1
            self.is_blocked = self._acks_pending > self._discard_limit

class Output:

    """
    A named output for a specific node.

    Note: This single output may be the source for multiple connections.
    """

    @classmethod
    def for_node(cls, node, config, reg_complete_cb, ack_complete_cb):
        """
        Factory method for all outputs corresponding to a particular node.

        Parameters:
            node (Node): The node associated with this input.
            config (NodeConfig): The node configuration containing all
                                 connections associated with this input.
            reg_complete_cb (function): A hook to be invoked when all
                                        registrations for this output have
                                        been received.
            ack_complete_cb (function): A hook to be invoked when all
                                        acknowledgements for this output
                                        have been received.

        Returns:
            A dictionary mapping each output name to an Output instance.
        """
        return {o.name : cls(node, o, config, reg_complete_cb, ack_complete_cb)
                for o in config.outputs}

    def __init__(self, node, output, config, reg_complete_cb, ack_complete_cb):
        """
        Initialises an instance of this class.

        Parameters:
            node (Node): The node associated with this input.
            config (NodeConfig): The node configuration containing all
                                 connections associated with this input.
            reg_complete_cb (function): A hook to be invoked when all
                                        registrations for this output have
                                        been received.
            ack_complete_cb (function): A hook to be invoked when all
                                        acknowledgements for this output
                                        have been received.
        """
        self._name = output.name
        if output.plugin:
            node.load_plugin(FileSystem.standard_project_dir, output.plugin)
        self._packet_type = node.find_type_by_name(output.type)
        self._logger = node.get_ros_node().get_logger()
        if output.external_topic is None:
            self._publishers = [_InternalPublisher(node, c,
                self._packet_type, self._reg_callback, self._ack_callback)
                for c in config.connections
                if c.source_node   == config.name and
                   c.source_output == self._name]
        else:
            self._publishers = [_ExternalPublisher(
                node, output, self._packet_type)]
        self._reg_complete_cb = reg_complete_cb
        self._ack_complete_cb = ack_complete_cb
        self.is_registered = all(p.is_registered for p in self._publishers)
        self.is_blocked = any(p.is_blocked for p in self._publishers)

    def check_blocked(self):
        """
        Check to see if any connection to a destination node has exceeded it's
        discard limit and should be consider blocked. Update the is_blocked
        member accordingly.
        """
        self.is_blocked = any(p.is_blocked for p in self._publishers)

    def _reg_callback(self):
        """
        A registration for a single destination node has been received.
        """
        wasnt_registered = not self.is_registered
        self.is_registered = all(p.is_registered for p in self._publishers)
        if self.is_registered and wasnt_registered:
            self._reg_complete_cb()

    def _ack_callback(self):
        """
        An acknowledgement for a single destination node has been received.
        """
        self.check_blocked()
        if not self.is_blocked:
            self._ack_complete_cb()

    def create_packet(self):
        """
        A factory method for creating packets of the type associated with this
        output.

        Returns:
           A new instance of the appropriate packet type.
        """
        return self._packet_type()

    def send(self, packet):
        """
        Send a packet to all destinations nodes.
        """
        for p in self._publishers:
            if p.is_blocked:
                self._logger.error(f"Dropped {self._name}")
            self._logger.info(f"Sending {self._name}")
            p.publish(packet)
        self.check_blocked()
