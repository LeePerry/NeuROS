# Copyright (c) 2023 Lee Perry

import argparse
import json
import os
import pathlib
import sys

from src.neuros.neuros.config import ConnectionConfig, NodeConfig

class CommandLineInterface(argparse.ArgumentParser):

    def error(self, message):
        self.print_help()
        print(f"\n{message}!\n")
        sys.exit(1)

class ProjectConfig:

    default_container = "osrf/ros:foxy-desktop"

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
    def cli_args(cls):
        parser = CommandLineInterface(
            formatter_class=argparse.RawTextHelpFormatter,
            description="NeuROS\n\n" +
                "An Integration Framework for Heterogenous Systems Neuroscience")

        parser.add_argument("-p",
                            "--project_path",
                            type=str,
                            required=True,
                            help="The path to the project configuration json file")
        parser.add_argument("-n",
                            "--node",
                            type=str,
                            required=False,
                            action="append",
                            help="[OPTIONAL] " +
                                "The name of a node as specified in the project configuration. " +
                                "Can be specified multiple times to launch a subset. "+
                                "Default is to launch all nodes.")
        args = parser.parse_args()

        if not os.path.isfile(args.project_path):
            parser.print_help()
            print(f"\nProject path '{args.project_path}' is not a file!\n")
            sys.exit(1)

        if args.node:
            args.node = set(args.node)

        return args

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
        self.container = ProjectConfig.default_container
        self.workspace_dir = data["workspace_directory"]
        self.project_dir = data["project_directory"]
        self.nodes = {n["name"] : NodeConfig(n,
            ConnectionConfig.filter_by_node(n["name"], data.get("connections", [])))
            for n in data["nodes"]}
