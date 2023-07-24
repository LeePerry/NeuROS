# Copyright (c) 2023 Lee Perry

import json
import os

class FileSystem:

    standard_project_dir = "/home/neuros/project"
    standard_workspace_dir = "/home/neuros/workspace"
    standard_node_dir  = "/home/neuros/node"
    standard_node_name = "node.json"
    standard_node_path = f"{standard_node_dir}/{standard_node_name}"

class InputConfig:

    def __init__(self, data):
        self.name = data["name"]
        self.type = data["type"]
        self.plugin = data.get("plugin")
        self.external_topic = data.get("external_topic")
        self.gazebo_type = data.get("gazebo_type")

class OutputConfig:

    def __init__(self, data):
        self.name = data["name"]
        self.type = data["type"]
        self.plugin = data.get("plugin")
        self.external_topic = data.get("external_topic")
        self.gazebo_type = data.get("gazebo_type")

class ConnectionConfig:

    @classmethod
    def filter_by_node(cls, name, data):
        return [cls(c) for c in data
                if (name == c["source_node"]) or (name == c["destination_node"])]

    def __init__(self, data):
        self.raw_data = data
        self.source_node = data["source_node"]
        self.source_output = data["source_output"]
        self.destination_node = data["destination_node"]
        self.destination_input = data["destination_input"]
        self.discard_limit = data.get("discard_limit")
        self.is_reliable = (self.discard_limit is not None)

class NodeConfig:

    @classmethod
    def from_standard_node_dir(cls):
        with open(FileSystem.standard_node_path, 'r') as f:
            data = json.load(f)
            node = data["node"]
            connections = [ConnectionConfig(c) for c in data["connections"]]
            return cls(node, connections)

    def __init__(self, data, connections):
        self.name = data["name"]
        self.plugin = data["plugin"]
        self.container = data["container"]
        self.inputs = [InputConfig(i) for i in data.get("inputs", [])]
        self.outputs = [OutputConfig(o) for o in data.get("outputs", [])]
        self.connections = connections
        self.raw_data = {
            "node" : data,
            "connections" : [c.raw_data for c in connections]}

    def write_to(self, node_dir):
        full_path = os.path.join(node_dir, FileSystem.standard_node_name)
        with open(full_path, 'w') as f:
            f.write(json.dumps(self.raw_data))
