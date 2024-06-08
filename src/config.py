# Copyright (c) 2023 Lee Perry

"""
This modules provides a centralised system-wide configuration for all NeuROS
funcionality, which can be customised via a command-line interface.
"""

import argparse
import json
import os
import pathlib
import sys

from src.validate import validate_project
from src.neuros.neuros.config import ConnectionConfig, FileSystem, NodeConfig

class CommandLineInterface(argparse.ArgumentParser):
    """
    This class specialises the ArgumentParser, allowing us to catch
    command-line argument parsing errors and apply custom handling.
    """

    def error(self, message):
        """
        The handler for a cli parsing error, which prints the full help
        information followed by the specific error message and exits with
        an error code.

        Parameters:
            message (str): The specific error message as generated by
                           the default ArgumentParser.
        """
        self.print_help()
        print(f"\n{message}!\n")
        sys.exit(1)

class ProjectConfig:
    """
    A class for representing a single project configuation.

    Attributes:
        default_container (str): Defines the base ROS2 container upon which all
                                 other components are built.
    """

    default_container = "osrf/ros:humble-desktop"

    @classmethod
    def build_environment(cls):
        """
        Creates a minimalist project config which can be used during the build
        and installation process.
        """
        workspace_dir = pathlib.Path(__file__).parent.parent.resolve()
        return cls({
            "name" : "build-environment",
            "nodes" : [],
            "connections" : [],
            "workspace_directory" : workspace_dir,
            "project_directory" : workspace_dir})

    @classmethod
    def cli_args(cls):
        """
        Parses the command-line arguments, validates that the supplied
        arguments are mutually compatible and applies reasonable defaults.

        The full command-line interface documentation can be found by running
            ./launch.py --help
        """
        parser = CommandLineInterface(
            formatter_class=argparse.RawTextHelpFormatter,
            description="NeuROS\n\n" +
                "An Integration Framework for Heterogenous Systems Neuroscience")

        parser.add_argument("-p",
                            "--project",
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
                                "Can be specified multiple times to launch a subset. " +
                                "Default is to launch all nodes.")
        parser.add_argument("-d",
                            "--domain-id",
                            default=0,
                            help="[OPTIONAL] " +
                                "Domain ID for logical separation of nodes. " +
                                "Defaults to 0 if not specified.")
        parser.add_argument("-v",
                            "--verbose",
                            required=False,
                            action="store_true",
                            help="[OPTIONAL] " +
                                "Launch ROS2 with debug level logging. " +
                                "Defaults to false when not specified.")
        parser.add_argument("-r",
                            "--record",
                            required=False,
                            action="store_true",
                            help="[OPTIONAL] " +
                                "Record all ROS topics to a rosbag in the project directory. " +
                                "Defaults to false when not specified.")
        parser.add_argument("-t",
                            "--topic",
                            type=str,
                            required=False,
                            action="append",
                            help="[OPTIONAL] " +
                                "Filter recording to a specific topic. " +
                                "Can be repeated multiple times to record multiple topics. " +
                                "Default is to record all topics.")
        parser.add_argument("-m",
                            "--monitor-system-load",
                            required=False,
                            action="store_true",
                            help="[OPTIONAL] " +
                                "Periodicially monitor and log system load for performance analysis.")
        parser.add_argument("-g",
                            "--node-graph",
                            required=False,
                            action="store_true",
                            help="[OPTIONAL] " +
                                "Display the ROS2 node connectivity graph (rqt_graph).")

        args = parser.parse_args()

        if args.node:
            args.node = set(args.node)

        error = None

        if not os.path.isfile(args.project):
            error = f"Project path '{args.project}' is not a file!"

        if args.topic and not args.record:
            error = "Topic specified but you're not recording!"

        if args.record and args.domain_id != 0:
            error = "Recording does not support domain IDs!"

        if error is not None:
            parser.print_help()
            print(f"\n{error}\n")
            sys.exit(1)

        return args

    @classmethod
    def from_file(cls, project_path):
        """
        Loads a ProjectConfig instance from a file path.

        Examples of such files can be found in the *examples* directory.

        Parameters:
            project_path (str): The path to a user-defined json file, which
                                defines all nodes and connections that
                                comprise a complete project.

        Returns:
            An instance of ProjectConfig which represents the project_path
            file contents.
        """
        workspace_dir = pathlib.Path(__file__).parent.parent.resolve()
        project_dir = pathlib.Path(project_path).parent.resolve()
        with open(project_path, 'r') as f:
            data = json.load(f)
        nodes = []
        for n in data["nodes"]:
            if isinstance(n, dict):
                _expand_paths(n)
                nodes.append(n)
            elif isinstance(n, str):
                with open(os.path.join(project_dir, n), 'r') as f:
                    external_node = json.load(f)
                    _expand_paths(external_node, os.path.dirname(n))
                    nodes.append(external_node)
            else:
                raise Exception(f"Unsupported node type {str(type(n))}: {n}")
        data["nodes"] = nodes
        validate_project(data)
        data["workspace_directory"] = workspace_dir
        data["project_directory"] = project_dir
        return cls(data)

    def __init__(self, data):
        """
        The class constructor, which extracts expected attributes from the
        user-defined project configuration.

        Parameters:
            data (dict) : The project json file represented as a Python
                          dictionary.
        """
        self.container = ProjectConfig.default_container
        self.workspace_dir = data["workspace_directory"]
        self.project_dir = data["project_directory"]
        self.nodes = {n["name"] : NodeConfig(n,
            ConnectionConfig.filter_by_node(n["name"], data.get("connections", [])))
            for n in data["nodes"]}

def _expand_paths(data, node_dir=""):
    """
    Expands any paths in the project config, such as relative node paths or
    environment variables that start with special characters.

    Parameters:
        data (dict) : The project json file represented as a Python dictionary.
        node_dir (str) : The node directory relative to the project directory.
    """
    plugin_path = data["plugin"]
    if not os.path.isabs(plugin_path):
        data["plugin"] = os.path.join(node_dir, plugin_path)
    env_vars = data.get("environment", {})
    for k, v in env_vars.items():
        v = str(v)
        if v.startswith('#'):
            v = os.path.join(FileSystem.standard_project_dir, node_dir, v[1:])
        if v.startswith('@'):
            v = os.path.join(FileSystem.standard_workspace_dir, v[1:])
        env_vars[k] = v
    data["environment"] = env_vars
