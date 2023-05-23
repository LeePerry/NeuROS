import json
import pathlib

from src.nodes.nodes.config import NodeConfig

class Config:

    @classmethod
    def from_file(cls, project_path):
        workspace_dir = pathlib.Path(__file__).parent.parent.resolve()
        project_dir = pathlib.Path(project_path).parent.resolve()
        with open(project_path, 'r') as f:
            data = json.load(f)
            data["workspace_directory"] = workspace_dir
            data["project_directory"] = project_dir
            return Config(data)

    def __init__(self, data):
        self.standard_workspace_dir = "/home/neuros/workspace"
        self.standard_project_dir = "/home/neuros/project"
        self.standard_node_dir = "/home/neuros/node"
        self.standard_container = "osrf/ros:foxy-desktop"
        self.name = data["name"]
        self.workspace_dir = data["workspace_directory"]
        self.project_dir = data["project_directory"]
        self._nodes = [NodeConfig(d) for d in data["nodes"]]

    def node_config_by_name(self, name):
        for n in self._nodes:
            if n.name == name:
                return n
        raise Exception(f"No node called {name}")

    def container_for_node(self, node):
        container = self.node_config_by_name(node).container
        return container if container else self.standard_container
