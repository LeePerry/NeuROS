from std_msgs.msg import String as String

from neuros.hooks import Hooks
from neuros.plugin import plugin_packet_type
from neuros.quality_of_service import standard_quality
from neuros.synchronisation import SynchronisationClient

class Receiver:

    @classmethod
    def for_node(cls, node):
        receivers = {}
        for connection in node.get_config().get_connections():
            if node.get_name() in connection.get_receivers():
                receivers[connection.get_name()] = cls(node, connection)
        return receivers

    def __init__(self, node, connection):
        self._node = node
        self._hooks = [h for n, h in Hooks.on_receive if n == connection.get_name()]
        self._data_subscription = self._node.create_subscription(
            plugin_packet_type(connection.get_packet_type_name()),
            f"{connection.get_sender()}/{connection.get_name()}/data",
            self._receive,
            standard_quality())
        self._latest_data = None
        self._fresh = False
        self._synchronisation = SynchronisationClient.for_connection(node, connection)

    def _receive(self, data):
        self._synchronisation.stop_registration()
        self._fresh = True
        self._latest_data = data
        for hook in self._hooks:
            hook(self._node)
            self._fresh = False
        self._synchronisation.acknowledge_packet()

    def is_fresh(self):
        return self._fresh

    def latest(self):
        return self._latest_data
