import json
import pathlib

from src.neuros.neuros.node_config import ConnectionConfig, NodeConfig

class ProjectConfig:

    @classmethod
    def build_environment(cls):
        workspace_dir = pathlib.Path(__file__).parent.parent.resolve()
        return cls({
            "name" : "build-environment",
            "nodes" : [],
            "connections" : [],
            "workspace_directory" : workspace_dir,
            "project_directory" : workspace_dir})

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
        self._name = data["name"]
        self._workspace_dir = data["workspace_directory"]
        self._project_dir = data["project_directory"]
        self._node_configs = [NodeConfig(n,
            ConnectionConfig.filter_by_node(
                n["name"], data.get("connections", [])))
            for n in data["nodes"]]

    def get_name(self):
        return self._name

    def get_workspace_dir(self):
        return self._workspace_dir

    def get_standard_workspace_dir(self):
        return "/home/neuros/workspace"

    def get_project_dir(self):
        return self._project_dir

    def get_standard_project_dir(self):
        return NodeConfig.standard_project_dir

    def get_standard_node_dir(self):
        return NodeConfig.standard_dir

    def get_all_nodes(self):
        return [n.get_name() for n in self._node_configs]

    def get_node_config_by_name(self, name):
        for n in self._node_configs:
            if n.get_name() == name:
                return n
        raise Exception(f"No node called {name}!")

    def get_container_for_node(self, node):
        container = self.get_node_config_by_name(node).get_container()
        return container if container else self.get_standard_container()

    def get_standard_container(self):
        return "osrf/ros:foxy-desktop"
