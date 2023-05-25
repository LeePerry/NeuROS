import json
import pathlib

from src.nodes.nodes.node_config import ConnectionConfig, NodeConfig

class ProjectConfig:

    @classmethod
    def build_environment(cls):
        workspace_dir = pathlib.Path(__file__).parent.parent.resolve()
        return cls({
            "name" : "build-environment",
            "nodes" : [],
            "connections" : [],
            "workspace_directory" : workspace_dir,
            "project_directory" : workspace_dir
        })

    @classmethod
    def from_file(cls, project_path):
        workspace_dir = pathlib.Path(__file__).parent.parent.resolve()
        project_dir = pathlib.Path(project_path).parent.resolve()
        with open(project_path, 'r') as f:
            data = json.load(f)
            data["workspace_directory"] = workspace_dir
            data["project_directory"] = project_dir
            return cls(data)

    def __init__(self, data):
        self.name = data["name"]
        print(f"Launching {self.name}...")
        self.standard_workspace_dir = "/home/neuros/workspace"
        self.standard_project_dir = NodeConfig.standard_project_dir
        self.standard_node_dir = NodeConfig.standard_dir
        self.standard_container = "osrf/ros:foxy-desktop"
        self.workspace_dir = data["workspace_directory"]
        self.project_dir = data["project_directory"]
        self._node_configs = [NodeConfig(n,
            ConnectionConfig.filter_by_node(n["name"], data["connections"]))
            for n in data["nodes"]]

    def get_node_config_by_name(self, name):
        for n in self._node_configs:
            if n.get_name() == name:
                return n
        raise Exception(f"No node called {name}")

    def get_container_for_node(self, node):
        container = self.get_node_config_by_name(node).get_container()
        return container if container else self.standard_container
