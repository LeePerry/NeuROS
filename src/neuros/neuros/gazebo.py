# Copyright (c) 2023 Lee Perry

import os

import subprocess

from neuros.config import FileSystem

class Gazebo:

    def __init__(self, world_path):
        if not os.path.isabs(world_path):
            world_path = os.path.join(
                FileSystem.standard_project_dir, world_path)
        #self._server = subprocess.Popen(
        #    f"ros2 launch gazebo_ros gzserver.launch.py world:={world_path}",
        #    shell=True)
        self._server = subprocess.Popen(
            f"gzserver --verbose -s libgazebo_ros_factory.so {world_path}",
            shell=True)

    def load_model(self, model_path):
        if not os.path.isabs(model_path):
            model_path = os.path.join(
                FileSystem.standard_project_dir, model_path)
        p = subprocess.run(
            f"ros2 run gazebo_ros spawn_entity.py -file {model_path} -entity robot",
            shell=True)
        if p.returncode != 0:
            raise Exception(f"Failed to load model, error code {p.returncode}")

    def list_topics(self):
        subprocess.run("ros2 topic list", shell=True)

    def shutdown(self):
        self._server.terminate()
