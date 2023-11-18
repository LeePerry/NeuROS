# Copyright (c) 2023 Lee Perry

"""
This module provides an interface to Gazebo simulator. It includes the ability
to load a world, spawn an instance of a robot and increment time in a strictly
controlled manner. Support is also provided for the Gazebo <-> ROS2 bridge, for
the purpose of automatic message type translation.

This module leverages the new Python bindings for gz-sim, details for which can
be found here https://github.com/gazebosim/gz-sim/tree/gz-sim7/python.
"""

import subprocess

from gz.common import set_verbosity as _set_verbosity
from gz.sim7 import TestFixture, world_entity, World

class Gazebo:

    """
    This class represents a running Gazebo instance, complete with a ROS2
    bridge and initialised with the specified world.
    """

    def __init__(self, node, world, verbosity=3):
        """
        Initialises a new instance of the Gazebo class.

        Parameters:
            node (Node): The NeuROS node associated with this instance.
            world (str): The path to the world SDF file.
            verbosity (int): The logging level for Gazebo log messages.
        """
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
        """
        Returns the full list of available Gazebo topics. This is not used
        directly by NeuROS but is included as a useful debugging tool for user
        plugins.

        Returns:
            A list of strings, where each string represents a single topic.
        """
        return [str(topic, "utf-8") for topic in
                subprocess.check_output("gz topic -l", shell=True).split()]

    def get_topic_info(self, topic):
        """
        Returns a human-readable description of a specific topic, including the
        supported message type. This is not used directly by NeuROS but is
        included as a useful debugging tool for user plugins.

        Note: There is a bug in some versions of Gazebo where topics that only
        exist due to subscribers are not included in the list. These topics are
        still available to both publish to and subscribe from, but will not be
        returned by this method.

        https://robotics.stackexchange.com/questions/104316/joint-position-controller-plugin-doesnt-subscribe-to-any-topics/104319#104319

        Parameters:
            topic (str): A valid ROS topic which needs to be inspected and
                         described.

        Returns:
            A string containing a human-readable description of the topic
            details.
        """
        return str(subprocess.check_output(
            f"gz topic -i -t {topic}", shell=True), "utf-8")

    def get_service_list(self):
        """
        Returns the full list of available Gazebo services. This is not used
        directly by NeuROS but is included as a useful debugging tool for user
        plugins.

        Returns:
            A list of strings, where each string represents a single service.
        """
        return [str(topic, "utf-8") for topic in
                subprocess.check_output("gz service -l", shell=True).split()]

    def spawn_entity(self, sdf_path, world="default", x=0, y=0, z=0, name=None):
        """
        Spawn a new entity (e.g. a robot) inside the specified world, at the
        specified location and with the specified name.

        Parameters:
            sdf_path (str): The path of the SDF file that describes the entity.
            world (str): The name of the world to load the entity into.
            x (int): The x position to spawn the entity at.
            y (int): The y position to spawn the entity at.
            z (int): The z position to spawn the entity at.
            name (str): The new name of the entity (if renaming is required).
        """
        xyz = f"x: {x} y: {y} z: {z}"
        command = ["gz", "service", "-s", f"/world/{world}/create",
                   "--reqtype", "gz.msgs.EntityFactory",
                   "--reptype", "gz.msgs.Boolean",
                   "--timeout", "1000",
                   "--req", f'sdf_filename: "{sdf_path}" ' +
                             'pose: {position{' + xyz + '}} ' +
                             'allow_renaming: true ' +
                           (f'name: "{name}"' if name else '')]
        response = subprocess.check_output(command).strip()
        return str(response, "utf-8") == "data: true"

    def step(self, count=1):
        """
        Step the simulation forward a specified numer of times. Note that the
        corresponding simulation time for each step is defined in the world SDF
        file (by the physics plugin) and will affect both the speed of the
        simulation and accuracy of the simulated physics. The number of steps
        the simulation is currently increment by can be found with the
        get_iterations() method.

        Parameters:
            count (int): The number of steps to increment the simulation by.
        """
        self._server.run(True, count, False)

    def get_world(self):
        """
        Getter for the Gazebo World object. Currently this is very limited, but
        as the Gazebo Python API grows new features will become available.

        Returns:
            The Gazebo API world instance.
        """
        self._require_started()
        return World(world_entity(self._ecm))

    def get_gravity(self):
        """
        Getter for the strength of gravity in the current simulation.

        Returns:
            The strength of gravity.
        """
        return self.get_world().gravity(self._ecm)

    def is_paused(self):
        """
        Check to see if the current simulation is paused or not.

        Returns:
            True if the simulation is paused, False otherwise.
        """
        self._require_started()
        return self._info.paused

    def get_iterations(self):
        """
        Getter for the number of steps the simulation is currently incremented
        by. This can be increased via the step() method.

        Returns:
            The number of steps the simulation is currently incremented by.
        """
        self._require_started()
        return self._info.iterations

    def get_real_time(self):
        """
        Getter for the amount of real-world time that has already elapsed
        during the simulation.

        Returns:
            The real-world time that has already elapsed.
        """
        self._require_started()
        return self._info.realtime

    def get_sim_time(self):
        """
        Getter for the amount of simulated time that has already elapsed during
        the simulation.

        Returns:
            The simulated time that has already elapsed.
        """
        self._require_started()
        return self._info.sim_time

    def get_entity_by_name(self, name):
        """
        Getter for any entity, found by searching the EntityComponentManager
        for the specified name.

        Parameters:
            name (str): The name of the entity to be found.

        Returns:
            The entity uniquely identified by the name parameter.
        """
        return self.get_world().model_by_name(self._ecm, name)

    def _store_state(self, info, ecm):
        """
        Stores the simulation information and EntityComponentManager ready
        to be queried by other dependant methods.

        Parameters:
            info: The simulation information.
            ecm (EntityComponentManager): The Gazebo entity component manager.
        """
        self._info = info
        self._ecm = ecm

    def _require_started(self):
        """
        Checks to ensure that the simulation has already completed at least one
        step, meaning this instance has access to the simulaton information
        and the EntityComponentManager. If not, an Exception will be raised.
        """
        if self._info is None or self._ecm is None:
            raise Exception("Simulation not started yet!")

    def _launch_bridge(self):
        """
        Launch the Gazebo <-> ROS2 bridge, translating any of this Nodes inputs
        or outputs which are defined as having a corresponding gazebo_type.

        Note that translations are unidirectional i.e. a NeuROS output packet
        type will only be translated for outgoing packets, a vice-versa for
        incoming packets.
        """
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
