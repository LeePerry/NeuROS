# Copyright (c) 2023 Lee Perry

import subprocess

from gz.common import set_verbosity as _set_verbosity
from gz.gazebo import TestFixture, world_entity, World

class Gazebo:

    def __init__(self, node, verbosity=4):
        _set_verbosity(verbosity)
        self._node = node

        scenario = TestFixture(node.get_parameter("world"))
        scenario.on_post_update(self._store_state)
        scenario.finalize()
        self._server = scenario.server()
        self._info = None
        self._ecm = None

        self.step(1)
        config = self._node._config # TODO don't access nodes private member
        command = ["ros2", "run", "ros_ign_bridge", "parameter_bridge"]
        command += [f"{i.external_topic}@{i.type.replace('.', '/')}@{i.gazebo_type}" # [
            for i in config.inputs
            if i.gazebo_type is not None]
        command += [f"{o.external_topic}@{o.type.replace('.', '/')}@{o.gazebo_type}" # ]
            for o in config.outputs
            if o.gazebo_type is not None]
        node.get_ros_node().get_logger().info(
            "Gazebo bridge: " + " ".join(command))
        self._bridge = subprocess.Popen(command)

    def _require_started(self):
        if self._info is None or self._ecm is None:
            raise Exception("Simulation not started yet!")

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
