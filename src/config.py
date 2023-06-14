# Copyright (c) 2023 Lee Perry

import json
import pathlib

from src.neuros.neuros.config import ConnectionConfig, NodeConfig

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
        self.workspace_dir = data["workspace_directory"]
        self.project_dir = data["project_directory"]
        self.nodes = {n["name"] : NodeConfig(n,
            ConnectionConfig.filter_by_node(n["name"], data.get("connections", [])))
            for n in data["nodes"]}
