# Copyright (c) 2023 Lee Perry

import json
import os

"""
This module provides node related configuration within NeuROS, such as the
standardised container filesystem, inputs, outputs, connections, and a complete
node configuration representation.
"""

class FileSystem:
    """
    This class provides standardised container filesystem paths.

    Attributes:
        standard_project_dir (str): The standard path of the project directory
                                    within a NeuROS container. The contents of
                                    which are equivalent to the parent
                                    directory of the currently loaded project
                                    file. All user provided dependencies must
                                    be contained within this directory.

        standard_workspace_dir (str): The standard path of the NeuROS workspace
                                      within a NeuROS container. This is
                                      equivalent to the NeuROS installation
                                      directory.

        standard_node_dir (str): The standard path of the node configuration
                                 directory within a NeuROS container. This is
                                 a temporary directory which exists only for
                                 the duration of the node execution. It
                                 contains a single json file, see
                                 standard_node_name below.

        standard_node_name (str): The standard name of the node configuration
                                  json file. This file is equivalent to a
                                  subset of the project file, containing only
                                  that information which relevant to this
                                  specific node.

        standard_node_path (str): The absolute path of the node configuration
                                  json file.
    """

    standard_project_dir = "/home/neuros/project"
    standard_workspace_dir = "/home/neuros/workspace"
    standard_node_dir  = "/home/neuros/node"
    standard_node_name = "node.json"
    standard_node_path = f"{standard_node_dir}/{standard_node_name}"

class InputConfig:

    """
    This class represents a single node input.

    Attributes:
        name (str): The name of the input, as must be specified by the
                    attribute of the same name in the project configuration
                    json file. This name must be unique and can then be used to
                    reference this input in connection definitions and user
                    specified plugins.

        type (str): The name of the packet type that can be received by this
                    input, as must be specified by the project attribute of the
                    same name in the project configuration json file.

        plugin (str): A (relative or absolute) path to a plugin, which can be
                      optionally specified by the attribute of the same name in
                      the project configuration json file. This may be used to
                      add support for custom message types.

        external_topic (str): An external topic, which can be optionally
                              specified by the attribute of the same name in
                              the project configuration json file. This may be
                              used to obtain data from outside NeuROS, such as
                              from a physics simulator.

        gazebo_type (str): This must be used in conjunction with external_topic
                           and is provided for convenience when using Gazebo.
                           This can be specified by the attribute of the same
                           name in the project configuration json file, and
                           will result in the Gazebo ROS2 bridge being launched
                           to automatically provide the gazebo_type-to-type
                           translation.
    """

    def __init__(self, data):
        """
        Initialises a new instance of the InputConfig class.

        Parameters:
            data (dict): The configuration settings for this instance.
        """
        self.name = data["name"]
        self.type = data["type"]
        self.plugin = data.get("plugin")
        self.external_topic = data.get("external_topic")
        self.gazebo_type = data.get("gazebo_type")

class OutputConfig:

    """
    This class represents a single node output.

    Attributes:
        name (str): The name of the output, as must be specified by the
                    attribute of the same name in the project configuration
                    json file. This name must be unique and can then be used to
                    reference this output in connection definitions and user
                    specified plugins.

        type (str): The name of the packet type that can be sent by this
                    output, as must be specified by the project attribute of the
                    same name in the project configuration json file.

        plugin (str): A (relative or absolute) path to a plugin, which can be
                      optionally specified by the attribute of the same name in
                      the project configuration json file. This may be used to
                      add support for custom message types.

        external_topic (str): An external topic, which can be optionally
                              specified by the attribute of the same name in
                              the project configuration json file. This may be
                              used to provide data outside of NeuROS, such as
                              to a physics simulator.

        gazebo_type (str): This must be used in conjunction with external_topic
                           and is provided for convenience when using Gazebo.
                           This can be specified by the attribute of the same
                           name in the project configuration json file, and
                           will result in the Gazebo ROS2 bridge being launched
                           to automatically provide the type-to-gazebo_type
                           translation.
    """

    def __init__(self, data):
        """
        Initialises a new instance of the OutputConfig class.

        Parameters:
            data (dict): The configuration settings for this instance.
        """
        self.name = data["name"]
        self.type = data["type"]
        self.plugin = data.get("plugin")
        self.external_topic = data.get("external_topic")
        self.gazebo_type = data.get("gazebo_type")

class ConnectionConfig:

    """
    This class represents a single connection between two nodes i.e. an
    input/output pair.

    Attributes:
        raw_data (dict): The data dictionary as loaded from the project
                         configuration json file, filtered for this specific
                         connection.

        source_node (str): The name of the source node that provides the data
                           required for this connection. This must be specified
                           by the attribute of the same name in the project
                           configuration json file.

        source_output (str): The name of the output of the source node that
                             outputs the data required for this node. This must
                             be specified by the attribute of the same name in
                             the project configuration json file.

        destination_node (str): The name of the destination node that consumed
                                the data passed through this connection. This
                                must be specified by the attribute of the same
                                name in the project configuration file.

        destination_input (str): The name of the input of the destination node
                                 that consumes the data passed through this
                                 connection. This must be specified by the
                                 attribute of the same name in the project
                                 configuration file.

        discard_limit (int): The number of consecutive packets that this
                             connection can drop before the source node becomes
                             blocked. Once blocked, the source node must wait
                             until the destination is ready to recieve data
                             again. For a maximally reliable connection this
                             should be set top 0. To allow infinite packets to
                             be dropped, don't specify this parameter at all
                             (i.e. None).

        is_reliable (boolean): This setting is inferred from the presence
                               (True) or absensce (False) of a discard_limit.
                               If True then the connection will use TCP, and
                               track the number of dropped packets. If False
                               the connection will use UDP and send data on a
                               "fire-and-forget" basis.
    """

    @classmethod
    def filter_by_node(cls, name, data):
        """
        Selects all connection configurations relevant to a particular node,
        uniquely identified by it's name.

        Returns:
            A list of ConnectionConfig instances where either the source_node,
            destination_node or both are equal to the specified name.
        """
        return [cls(c) for c in data
                if (name == c["source_node"]) or (name == c["destination_node"])]

    def __init__(self, data):
        """
        Initialises a new instance of the ConnectionConfig class.

        Parameters:
            data (dict): The configuration settings for this instance.
        """
        self.raw_data = data
        self.source_node = data["source_node"]
        self.source_output = data["source_output"]
        self.destination_node = data["destination_node"]
        self.destination_input = data["destination_input"]
        self.discard_limit = data.get("discard_limit")
        self.is_reliable = (self.discard_limit is not None)

class NodeConfig:

    """
    This class represents a single connection between two nodes i.e. an
    input/output pair.

    Attributes:
        name (str): The name of the node configured by this instance.

        plugin (str): The path to the user supplied Python file, containing
                      the plugin to be loaded for this node.

        container (str): The name of any docker container present on the system
                         to use to execute this node within. Note that this
                         container must minimally contain a ROS2 installation.
                         Many such container images are provided by NeuROS e.g.
                         neuros_python.

        inputs (list): The list of InputConfig instances relevant to this node.

        outputs (list): The list of OutputConfig instances relevant to this
                        node.

        env (dict): A dictionary of environment variables to inject into the
                    node environment before loading the plugin. Note that there
                    exists support for two special characters:
                        * An environment variable value starting with the
                          character '#' will have the first '#' automatically
                          replaced with the standard NeuROS project directory.
                          This can be used to express paths relative to the
                          project configuration file's parent directory.
                        * An environment variable value starting with the
                          character '@' will have the first '@' automatically
                          replaced with the standard NeuROS workspace
                          directory. This can be used to express paths relative
                          to the NeuROS installation directory.

        raw_data (dict): The raw representation of the json config, which is
                         used as a convenient method of rewriting it to a file.
    """

    @classmethod
    def from_standard_node_dir(cls):
        """
        Loads a NodeConfig instance from a json file located at the standard
        node path.

        Returns:
            The NodeConfig instance representing the node configuration.
        """
        with open(FileSystem.standard_node_path, 'r') as f:
            data = json.load(f)
            node = data["node"]
            connections = [ConnectionConfig(c) for c in data["connections"]]
            return cls(node, connections)

    def __init__(self, data, connections):
        """
        Initialises a new instance of the NodeConfig class.

        Parameters:
            data (dict): The configuration settings for this instance.
            connections (list): The list of ConnectionConfig instances relevant
                                to this node.
        """
        self.name = data["name"]
        self.plugin = data["plugin"]
        self.container = data["container"]
        self.inputs = [InputConfig(i) for i in data.get("inputs", [])]
        self.outputs = [OutputConfig(o) for o in data.get("outputs", [])]
        self.env = data.get("environment", {})
        self.connections = connections
        self.raw_data = {
            "node" : data,
            "connections" : [c.raw_data for c in connections]}

    def write_to(self, node_dir):
        """
        Writes a NodeConfig instance to a json file located at the standard
        node path.
        """
        full_path = os.path.join(node_dir, FileSystem.standard_node_name)
        with open(full_path, 'w') as f:
            f.write(json.dumps(self.raw_data))
