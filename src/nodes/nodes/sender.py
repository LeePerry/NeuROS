from nodes.plugin import plugin_packet_type
from nodes.quality_of_service import standard_quality
from nodes.synchronisation import SynchronisationServer

class Sender:

    @classmethod
    def for_node(cls, node):
        senders = {}
        for connection in node.get_config().get_connections():
            if node.get_name() == connection.get_sender():
                key = connection.get_name()
                if key in senders:
                    raise Exception(f"Connection {key} has multiple definitions!")
                senders[key] = cls(node, connection)
        return senders

    def __init__(self, node, connection):
        self._node = node
        self._packet_type = plugin_packet_type(connection.get_packet_type_name())
        self._publisher = self._node.create_publisher(
            self._packet_type,
            f"{connection.get_sender()}/{connection.get_name()}/data",
            standard_quality())
        self._synchronisation = SynchronisationServer.for_connection(node, connection)

    def create_packet(self):
        return self._packet_type()

    def send(self, data):
        self._synchronisation.pre_send()
        self._publisher.publish(data)
        self._synchronisation.post_send()
