# Copyright (c) 2023 Lee Perry

import time

from neuros.hooks import neuros_initialise, neuros_function, All

@neuros_initialise(outputs="polling_day")
def announce_election(node):
    node.get_ros_node().get_logger().info("Let's vote!")
    return node.make_packet("polling_day")

@neuros_function(inputs=All("vote"), outputs="polling_day")
def polling_station(node, votes):
    votes = [v.data for v in votes]
    vote_count = 1
    most_popular = "none"
    for candidate in set(votes):
        count = votes.count(candidate)
        if count > vote_count:
            vote_count = count
            most_popular = candidate
    node.get_ros_node().get_logger().info(f"The winner is '{most_popular}'")
    time.sleep(5)
    node.get_ros_node().get_logger().info("And again!")
    return node.make_packet("polling_day")
