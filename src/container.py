# Copyright (c) 2023 Lee Perry

import subprocess
import tempfile

from src.neuros.neuros.config import FileSystem

class Container:

    def __init__(self, config):
        self.config = config

    def docker_command(self, command, work_dir="", container="", node_dir=""):
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
        # build nodes (but ignore the whisk_eye plugin from examples directory)
        return self.docker_command("colcon build --symlink-install ", # + "--packages-ignore-regex whiskeye_plugin",
                                   work_dir=FileSystem.standard_workspace_dir,
                                   container="neuros_gazebo")

    def run_node(self, name):
        with tempfile.TemporaryDirectory() as temporary_dir:
            self.config.nodes[name].write_to(temporary_dir)
            self.docker_command("ros2 run neuros node",
                                container=self.config.nodes[name].container,
                                node_dir=temporary_dir)
