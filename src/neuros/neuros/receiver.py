# Copyright (c) 2023 Lee Perry

from std_msgs.msg import String as String

from neuros.hooks import Hooks
from neuros.plugin import plugin_packet_type
from neuros.quality_of_service import standard_quality
from neuros.synchronisation import SynchronisationClient

class HookAssociatedReceiverGroup:

    @classmethod
    def for_node(cls, node, receiver_groups):
        all_associations = []
        for names, hook in Hooks.on_receive_all:
            members = []
            for connection_name, group in receiver_groups.items():
                if connection_name in names:
                    members.append(group)
            all_associations.append(cls(node, hook, members))
        return all_associations

    def __init__(self, node, hook, members):
        self._groups = {m._connection_name : m for m in members}
        for group in self._groups.values():
            group._groups.append(self)
        self._satisfied = set()
        self._hook = hook
        self._node = node

    def check_satisfied(self, name):
        self._satisfied.add(name)
        if self._satisfied == self._groups.keys():
            self._hook(self._node)
            self._satisfied.clear()
            return True
        return False

class ReceiverGroup:

    def __init__(self, connection_name):
        self._groups = []
        self._connection_name = connection_name
        self._receivers = {}
        self._data = {}
        self._fresh = True
        self._latest_data = None

    def _add_receiver(self, sender_name, receiver):
        receiver._group = self
        self._receivers[sender_name] = receiver

    def _received(self, sender_name, data):
        self._fresh = True
        self._latest_data = data
        self._data[sender_name] = data
        if self._data.keys() == self._receivers.keys():
            for group in self._groups:
                if group.check_satisfied(self._connection_name):
                    self._fresh = False
            self._data.clear()

    def get_latest(self):
        return self._latest_data

    def get_all(self):
        return self._data.values()

    def is_fresh(self):
        return self._fresh

class Receiver:

    @classmethod
    def for_node(cls, node):
        receiver_groups = {}
        for connection in node.get_config().get_connections():
            if node.get_name() in connection.get_receivers():
                connection_name = connection.get_name()
                if connection_name not in receiver_groups:
                    receiver_groups[connection_name] = ReceiverGroup(connection_name)
                receiver_groups[connection.get_name()]._add_receiver(
                    connection.get_sender(), cls(node, connection))
        hook_associations = HookAssociatedReceiverGroup.for_node(node, receiver_groups)
        return (receiver_groups, hook_associations)

    def __init__(self, node, connection):
        self._node = node
        self._group = None
        self._sender = connection.get_sender()
        self._data_subscription = self._node.create_subscription(
            plugin_packet_type(connection.get_packet_type_name()),
            f"{connection.get_sender()}/{connection.get_name()}/data",
            self._receive,
            standard_quality(),
            callback_group=node.get_hook_callback_group())
        self._synchronisation = SynchronisationClient.for_connection(
            node, connection)
        self._hooks = Hooks.filter_by_connection(
            Hooks.on_receive_any, connection.get_name())

    def _receive(self, data):
        self._synchronisation.stop_registration()
        self._group._received(self._sender, data)
        for hook in self._hooks:
            hook(self._node)
        self._synchronisation.acknowledge_packet()
