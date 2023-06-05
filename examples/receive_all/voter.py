# Copyright (c) 2023 Lee Perry

import random

from neuros.hooks import neuros_receive_any

@neuros_receive_any("polling_day")
def place_vote(node):
    vote = node.get_outgoing("vote")
    choice = vote.create_packet()
    choice.data = random.choice(["red", "blue"])
    node.get_logger().info(f"Voting for {choice.data}")
    vote.send(choice)
