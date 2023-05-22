# Copyright (c) 2023 Lee Perry

"""
This module encapsulates the execution of NeuROS commands inside a docker
container.

Commands suuported include those for building projects and running nodes.
"""

import subprocess
import tempfile
import time

from src.config import ProjectConfig
from src.neuros.neuros.config import FileSystem

class Container:
    """
    A class to represent a configured container, upon which methods can be
    invoked in order to launch and execute commands. Note that the associated
    docker container only exists during the execution of one this class'
    methods.
    """

    def __init__(self, config):
        """
        Creates an instance of a fully configured NeuROS container, which
        remains in an un-launched state.
        """
        self.config = config

    def docker_command(self, command, work_dir="", container="", node_dir=""):
        """
        A generalised method capable of executing any command within the
        running container.

        Parameters:
            command (str) : A Bash command to be executed within the running
                            container.
            work_dir (str) : The working directory to execute the command from.
            container (str) : The name of the container image to use (defaults
                              to config.container).
            node_dir (str) : The full path of a directory to be mounted as the
                             NeuROS node directory (defaults to not mounting any
                             directory).

        Returns:
            The exit code returned by the command.
        """
        full_command = ["docker", "run"]
        full_command += ["-v", f"{self.config.workspace_dir}:" +
                               f"{FileSystem.standard_workspace_dir}",
                         "-v", f"{self.config.project_dir}:" +
                               f"{FileSystem.standard_project_dir}"]
        if node_dir:
            full_command += ["-v", f"{node_dir}:" +
                                   f"{FileSystem.standard_node_dir}"]
        full_command += ["-it", "--init", "--rm", "--platform", "linux/amd64"]
        if work_dir:
            full_command += ["--workdir", work_dir]
        if not container:
            container = self.config.container
        full_command += [container]
        full_command += ["bash", "-c", command]
        return subprocess.run(full_command)

    def build_workspace(self):
        """
        Builds the complete NeuROS workspace with symlink installation.
        This is intended to be invoked only during installation.

        Returns:
            An exit code indicating the success/failure of this action.
        """
        return self.docker_command("colcon build --symlink-install " +
                                   "--packages-ignore-regex whiskeye",
                                   work_dir=FileSystem.standard_workspace_dir,
                                   container=ProjectConfig.default_container)

    def build_examples(self):
        """
        Builds all NeuROS examples with symlink installation.
        This is intended to be invoked only during installation.

        Returns:
            An exit code indicating the success/failure of this action.
        """
        return self.docker_command("colcon build --symlink-install " +
                                   "--packages-select-regex whiskeye",
                                   work_dir=FileSystem.standard_workspace_dir,
                                   container="neuros_gazebo")

    def build_documentation(self):
        """
        Generates all NeuROS documentation and places it in the docs directory.
        This is intended to be invoked only during installation.

        Returns:
            An exit code indicating the success/failure of this action.
        """
        docs_dir = FileSystem.standard_workspace_dir + "/docs"
        return self.docker_command("make html",
                                   work_dir=docs_dir,
                                   container="neuros_gazebo")

    def run_node(self, name, verbose=False, domain_id=0):
        """
        Launches a NeuROS node identified from the container config via the
        unique name.
        This is intended to be invoked during the execution of a NeuROS
        project.

        Parameters:
            name (str) : The uniquely identifying name of the node.
            verbose (boolean) : Indicates whether to launch the node with
                                debug-level logging (defaults to False).
            domain_id (int) : Provides logical separation of NeuROS nodes
                              via the associated --ros-domain-id functionality
                              (defaults to 0).

        Returns:
            An exit code indicating the success/failure of this action.
        """
        with tempfile.TemporaryDirectory() as temporary_dir:
            self.config.nodes[name].write_to(temporary_dir)
            extra = " --ros-args --log-level DEBUG" if verbose else ""
            return self.docker_command(
                f"ros2 run neuros node --ros-domain-id {domain_id} {extra}",
                container=self.config.nodes[name].container,
                node_dir=temporary_dir)

    def record_topics(self, topics=[]):
        """
        Launches a container capable of recording all messages sent over
        a subset of topics and storing them on disk. This leverages the
        associated "ros2 bag record" functionality.  The resulting data is
        stored in the following directory:
            <project_dir>/neuros_recording_<timestamp>

        Parameters:
            topics (list) : A subset of topics to record (defaults to all).

        Returns:
            An exit code indicating the success/failure of this action.
        """
        gmt = time.gmtime()
        rid = time.strftime("%Y_%m_%d_%H_%M_%S", gmt)
        return self.docker_command(
            f"ros2 bag record -o " +
            f"{FileSystem.standard_project_dir}/neuros_recording_{rid} " +
            (" ".join(topics) if topics else "-a"),
            container=ProjectConfig.default_container)
