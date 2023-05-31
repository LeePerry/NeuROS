import subprocess
import tempfile

class Container:

    def __init__(self,
                 config,
                 run=subprocess.run):
        self._config = config
        self._run = run

    def _docker_command(self, container, command, node_dir=""):
        full_command = ["docker", "run"]
        full_command += ["-v", f"{self._config.workspace_dir}:" +
                               f"{self._config.standard_workspace_dir}",
                         "-v", f"{self._config.project_dir}:" +
                               f"{self._config.standard_project_dir}"]
        if node_dir:
            full_command += ["-v", f"{node_dir}:" +
                                   f"{self._config.standard_node_dir}"]
        full_command += ["-it"] # this handles ctrl-c (but only appears to work for the first container)
        full_command += [container]
        full_command += ["bash", "-c", command]
        self._run(full_command)

    def build_workspace(self):
        print("Building workspace")
        self._docker_command(self._config.standard_container,
            f"cd {self._config.standard_workspace_dir}; " +
             "colcon build --symlink-install")

    def run_node(self, name):
        with tempfile.TemporaryDirectory() as temporary_dir:
            self._config.get_node_config_by_name(name).write_to(temporary_dir)
            self._docker_command(self._config.get_container_for_node(name),
                                f"cd {self._config.standard_workspace_dir}; " +
                                ". install/setup.bash > /dev/null 2>&1; " +
                                f"ros2 run nodes node",
                                node_dir=temporary_dir)
