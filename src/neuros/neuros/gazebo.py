# Copyright (c) 2023 Lee Perry

import os
import subprocess

from gz.common import set_verbosity as _set_verbosity
from gz.sim7 import TestFixture, world_entity, World

from neuros.config import FileSystem

class Gazebo:

    def __init__(self, node, world, verbosity=3):
        _set_verbosity(verbosity)
        scenario = TestFixture(world)
        scenario.on_post_update(self._store_state)
        scenario.finalize()
        self._node = node
        self._server = scenario.server()
        self._info = None
        self._ecm = None
        self.step(1)
        self._launch_bridge()

    def get_topic_list(self):
        return [str(topic, "utf-8") for topic in
                subprocess.check_output("gz topic -l", shell=True).split()]

    def get_topic_info(self, topic):
        return str(subprocess.check_output(
            f"gz topic -i -t {topic}", shell=True), "utf-8")

    def get_service_list(self):
        return [str(topic, "utf-8") for topic in
                subprocess.check_output("gz service -l", shell=True).split()]

    def spawn_entity(self, sdf_path, world="default", x=0, y=0, z=0, name=None):
        xyz = f"x: {x} y: {y} z: {z}"
        command = ["gz", "service", "-s", f"/world/{world}/create",
                   "--reqtype", "gz.msgs.EntityFactory",
                   "--reptype", "gz.msgs.Boolean",
                   "--timeout", "500",
                   "--req", f'sdf_filename: "{sdf_path}" ' +
                             'pose: {position{' + xyz + '}} ' +
                             'allow_renaming: true ' +
                           (f'name: "{name}"' if name else '')]
        response = subprocess.check_output(command).strip()
        return str(response, "utf-8") == "data: true"

    def step(self, count=1):
        self._server.run(True, count, False)

    def get_world(self):
        self._require_started()
        return World(world_entity(self._ecm))

    def get_gravity(self):
        return self.get_world().gravity(self._ecm)

    def is_paused(self):
        self._require_started()
        return self._info.paused

    def get_iterations(self):
        self._require_started()
        return self._info.iterations

    def get_real_time(self):
        self._require_started()
        return self._info.realtime

    def get_sim_time(self):
        self._require_started()
        return self._info.sim_time

    def get_entity_by_name(self, name):
        return self.get_world().model_by_name(self._ecm, name)

    def _store_state(self, info, ecm):
        self._info = info
        self._ecm = ecm

    def _require_started(self):
        if self._info is None or self._ecm is None:
            raise Exception("Simulation not started yet!")

    def _launch_bridge(self):
        command = ["ros2", "run", "ros_gz_bridge", "parameter_bridge"]
        inputs = [f"{i.external_topic}@{i.type.replace('.', '/')}[{i.gazebo_type}"
            for i in self._node._config.inputs
            if i.gazebo_type is not None]
        outputs = [f"{o.external_topic}@{o.type.replace('.', '/')}]{o.gazebo_type}"
            for o in self._node._config.outputs
            if o.gazebo_type is not None]
        self._bridge = (subprocess.Popen(command + inputs + outputs)
                        if inputs or outputs
                        else None)

    def _full_path(self, relative_path):
        if os.path.isabs(relative_path):
            return relative_path
        project_path = os.path.join(FileSystem.standard_project_dir, relative_path)
        if os.path.exists(project_path):
            return project_path
        resource_dir = os.environ.get("GZ_SIM_RESOURCE_PATH")
        if resource_dir:
            resource_path = os.path.join(resource_dir, relative_path)
            if os.path.exists(resource_path):
                return resource_path
        return relative_path
