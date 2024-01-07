# Copyright (c) 2023 Lee Perry

"""
This example demonstrates receiving the same input from multiple source nodes,
by simulating a democratic voting process.
"""

import random

from neuros.hooks import neuros_function

@neuros_function(inputs="polling_day", outputs="vote")
def place_vote(node, _):
    """
    This plugin code is executed by three separate nodes simultaneously.

    Upon receipt of a polling day packet the voter node makes a random choice
    and responds with a vote packet.
    """
    vote = node.make_packet("vote")
    vote.data = random.choice(["red", "blue"])
    node.get_ros_node().get_logger().info(f"Voting for {vote.data}")
    return vote
