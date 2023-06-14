# Copyright (c) 2023 Lee Perry

from neuros.config import FileSystem

class Publisher:

    def __init__(self, node, connection, packet_type, reg_cb, ack_cb):
        self.is_registered = False
        self.is_blocked = False
        self._discard_limit = connection.discard_limit
        self._acks_pending = 0
        self._reg_cb = reg_cb
        self._ack_cb = ack_cb
        self._data_pub, self._ack_sub, self._reg_sub = node.make_neuros_output(
            connection, packet_type, self._ack_callback, self._reg_callback)

    def _reg_callback(self, _):
        self.is_registered = True
        self._reg_cb()

    def _ack_callback(self, _):
        self.is_blocked = False
        self._acks_pending = 0
        self._ack_cb()

    def publish(self, packet):
        self._data_pub.publish(packet)
        self._acks_pending += 1
        self.is_blocked = self._acks_pending > self._discard_limit

class Output:

    @classmethod
    def for_node(cls, node, config, reg_complete_cb, ack_complete_cb):
        return {o.name : cls(node, o, config, reg_complete_cb, ack_complete_cb)
                for o in config.outputs}

    def __init__(self, node, output, config, reg_complete_cb, ack_complete_cb):
        self._name = output.name
        if output.plugin:
            node.load_neuros_plugin(
                FileSystem.standard_project_dir, output.plugin)
        self._packet_type = node.find_neuros_type_by_name(output.type)
        self._logger = node.get_logger()
        self._publishers = [Publisher(node, c, self._packet_type,
                                      self._reg_callback, self._ack_callback)
            for c in config.connections if c.source_node == config.name]
        self._reg_complete_cb = reg_complete_cb
        self._ack_complete_cb = ack_complete_cb
        self.is_registered = False
        self.is_blocked = False

    def _reg_callback(self):
        wasnt_registered = not self.is_registered
        self.is_registered = all(p.is_registered for p in self._publishers)
        if self.is_registered and wasnt_registered:
            self._reg_complete_cb()

    def _ack_callback(self):
        self.is_blocked = any(p.is_blocked for p in self._publishers)
        if not self.is_blocked:
            self._ack_complete_cb()

    def create_packet(self):
        return self._packet_type()

    def send(self, packet):
        for p in self._publishers:
            if p.is_blocked:
                self._logger.warning(f"Dropped {self._name} packet")
            p.publish(packet)
        self.is_blocked = any(p.is_blocked for p in self._publishers)
