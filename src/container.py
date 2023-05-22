import pathlib
import subprocess

class Container:

    def __init__(self, config, name):
        self._config = config
        self._name = name

    def run_and_wait(self):
        neuros_root = pathlib.Path(__file__).parent.parent.resolve()
        setup_command = ". $NEUROS/install/setup.bash > /dev/null 2>&1"
        run_command = ros2 run neuros_simulation

        subprocess.run([])


NEUROS="/home/NeuROS_prototype"
SETUP=". $NEUROS/install/setup.bash > /dev/null 2>&1"
RUN="ros2 run neuros_simulation"
docker run \
    -v `pwd`:$NEUROS \
    -it osrf/ros:foxy-desktop \
    bash -c "$SETUP; $RUN $1"

