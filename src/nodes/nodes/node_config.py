import json
import os

# Note: Cannot import neuros files, as this config is used in 2 different
# contexts.
# i.e. all dependencies must be contained within this file.

class ConnectionConfig:

    @classmethod
    def filter_by_node(cls, name, data):
        return [cls(c) for c in data
                if c["sender"] == name or name in c["receivers"]]

    def __init__(self, data):
        self._raw_data = data
        self._name = data["name"]
        self._packet_type = data["packet_type"]
        self._sender = data["sender"]
        self._receivers = data["receivers"]
        self._synchronisation = data.get("synchronisation", "null")
        self._max_permitted_no_ack = data.get("max_permitted_no_ack", 0)

    def get_name(self):
        return self._name

    def get_packet_type_name(self):
        return self._packet_type

    def get_sender(self):
        return self._sender

    def get_receivers(self):
        return self._receivers

    def get_synchronisation(self):
        return self._synchronisation

    def get_max_permitted_no_ack(self):
        return self._max_permitted_no_ack

class NodeConfig:

    standard_dir  = "/home/neuros/node"
    standard_name = "node.json"
    standard_path = f"{standard_dir}/{standard_name}"
    standard_project_dir = "/home/neuros/project"

    @classmethod
    def from_standard(cls):
        with open(cls.standard_path, 'r') as f:
            data = json.load(f)
            node = data["node"]
            connections = [ConnectionConfig(c) for c in data["connections"]]
            return cls(node, connections)

    def __init__(self, data, connections):
        self._name = data["name"]
        self._source = data["source"]
        self._container = data.get("container", "osrf/ros:foxy-desktop")
        self._connections = connections
        self._raw_data = {
            "node" : data,
            "connections" : [c._raw_data for c in connections]}

    def get_name(self):
        return self._name

    def get_source(self):
        return self._source

    def get_container(self):
        return self._container

    def get_connections(self):
        return self._connections

    def get_parameter(self, name):
        return self._raw_data["node"].get(name)

    def write_to(self, node_dir):
        with open(os.path.join(node_dir, NodeConfig.standard_name), 'w') as f:
            f.write(json.dumps(self._raw_data))
