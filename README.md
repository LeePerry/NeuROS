# NeuROS

An Integration Framework for Heterogeneous Computational Systems Neuroscience

MIT License

Copyright (c) 2023 Lee Perry

This framework makes it trivial to define, implement and execute containerised
Python ROS2 nodes inside Docker. Images are also provided for several industry
standard tools, such as Gazebo and NEST.

To build and install NeuROS and it's dependencies, simply run:
    ./install.py

Several example projects are provided in the examples directory. These can be
launched with e.g.:
    ./launch.py --project ./examples/tennis/tennis.json
