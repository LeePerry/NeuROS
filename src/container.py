# Copyright (c) 2023 Lee Perry

import subprocess
import tempfile

from src.neuros.neuros.config import FileSystem

class Container:

    def __init__(self, config):
        self.config = config

    def docker_command(self, command, container="", node_dir=""):
        full_command = ["docker", "run"]
        full_command += ["-v", f"{self.config.workspace_dir}:" +
                               f"{FileSystem.standard_workspace_dir}",
                         "-v", f"{self.config.project_dir}:" +
                               f"{FileSystem.standard_project_dir}"]
        if node_dir:
            full_command += ["-v", f"{node_dir}:" +
                                   f"{FileSystem.standard_node_dir}"]
        full_command += ["--workdir", FileSystem.standard_workspace_dir]
        full_command += ["-it", "--init", "--rm"]
        if not container:
            container = self.config.container
        full_command += [container]
        full_command += ["bash", "-c", command]
        return subprocess.run(full_command)

    def build_workspace(self):
        return self.docker_command("colcon build --symlink-install")

    def list_executables(self):
        return self.docker_command(". install/setup.bash > /dev/null 2>&1; " +
                                   "exec ros2 pkg executables neuros")

    def run_node(self, name):
        with tempfile.TemporaryDirectory() as temporary_dir:
            self.config.nodes[name].write_to(temporary_dir)
            self.docker_command(". install/setup.bash > /dev/null 2>&1; " +
                                "exec ros2 run neuros node",
                                container=self.config.nodes[name].container,
                                node_dir=temporary_dir)
