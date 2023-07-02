# Copyright (c) 2023 Lee Perry

import random

from neuros.hooks import neuros_function

@neuros_function(inputs="polling_day", outputs="vote")
def place_vote(node, _):
    vote = node.make_packet("vote")
    vote.data = random.choice(["red", "blue"])
    node.get_logger().info(f"Voting for {vote.data}")
    return vote
