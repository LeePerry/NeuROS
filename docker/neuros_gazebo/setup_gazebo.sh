#!/bin/bash
set -e

. "/opt/ros/foxy/setup.bash"
. "/home/neuros/workspace/install/setup.bash"
. "/usr/share/gazebo/setup.sh"

export IGN_GAZEBO_RESOURCE_PATH="\
/home/neuros/project:\
/opt/ros/foxy/share/gazebo_plugins/worlds:\
/usr/share/gazebo-11/worlds:\
/usr/share/gazebo-11/models:\
${IGN_GAZEBO_RESOURCE_PATH}"

export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=$LD_LIBRARY_PATH

exec "$@"
