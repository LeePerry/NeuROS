# Copyright (c) 2023 Lee Perry

import subprocess
import tempfile

class Container:

    def __init__(self, config):
        self._config = config

    def _docker_command(self, command, container="", node_dir=""):
        full_command = ["docker", "run"]
        full_command += ["-v", f"{self._config.get_workspace_dir()}:" +
                               f"{self._config.get_standard_workspace_dir()}",
                         "-v", f"{self._config.get_project_dir()}:" +
                               f"{self._config.get_standard_project_dir()}"]
        if node_dir:
            full_command += ["-v", f"{node_dir}:{self._config.get_standard_node_dir()}"]
        full_command += ["--workdir", self._config.get_standard_workspace_dir()]
        full_command += ["-it", "--init", "--rm"]
        if not container:
            container = self._config.get_standard_container()
        full_command += [container]
        full_command += ["bash", "-c", command]
        return subprocess.run(full_command)

    def build_workspace(self):
        return self._docker_command("colcon build --symlink-install")

    def list_executables(self):
        return self._docker_command(". install/setup.bash > /dev/null 2>&1; " +
                                    "exec ros2 pkg executables neuros")

    def run_node(self, name):
        with tempfile.TemporaryDirectory() as temporary_dir:
            self._config.get_node_config_by_name(name).write_to(temporary_dir)
            self._docker_command(". install/setup.bash > /dev/null 2>&1; " +
                                 "exec ros2 run neuros node",
                                 container=self._config.get_container_for_node(name),
                                 node_dir=temporary_dir)
