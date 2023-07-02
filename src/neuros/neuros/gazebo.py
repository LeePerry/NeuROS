# Copyright (c) 2023 Lee Perry

import os

from ignition.common import set_verbosity as _set_verbosity
from ignition.gazebo import TestFixture

# https://github.com/gazebosim/gz-sim/blob/ign-gazebo6/examples/scripts/python_api/testFixture.py

class Gazebo:

    def __init__(self, world_path, verbosity=4):
        _set_verbosity(verbosity)
        self._helper = TestFixture(world_path)
        # add any required callbacks here
        self._helper.finalize()
        self._server = self._helper.server()

    def run(self):
        self._server.run(True, 1000, False)
