import json
import os

class NodeConfig:

    @classmethod
    def from_standard(cls):
        with open("/home/neuros/node/node.json", 'r') as f:
            return NodeConfig(json.load(f))

    def __init__(self, data):
        self.raw_data = data
        self.name = data["name"]
        self.implementation = data["implementation"]
        self.container = data.get("container")

    def write_to(self, node_dir):
        with open(os.path.join(node_dir, "node.json"), 'w') as f:
            f.write(json.dumps(self.raw_data))
