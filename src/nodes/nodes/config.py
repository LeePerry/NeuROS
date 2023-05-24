import json
import os

class NodeConfig:

    standard_dir  = "/home/neuros/node"
    standard_name = "node.json"
    standard_path = f"{standard_dir}/{standard_name}"
    standard_project_dir = "/home/neuros/project"

    @classmethod
    def from_standard(cls):
        with open(NodeConfig.standard_path, 'r') as f:
            return NodeConfig(json.load(f))

    def __init__(self, data):
        self.raw_data = data
        node = data["node"]
        self.name = node["name"]
        self.implementation = node["implementation"]
        self.container = node.get("container")

    def write_to(self, node_dir):
        with open(os.path.join(node_dir, NodeConfig.standard_name), 'w') as f:
            f.write(json.dumps(self.raw_data))
