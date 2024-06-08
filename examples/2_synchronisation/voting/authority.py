# Copyright (c) 2023 Lee Perry

"""
This example demonstrates receiving the same input from multiple source nodes,
by simulating a distributed voting process.
"""

from neuros.hooks import neuros_initialise, neuros_function, All

@neuros_initialise(outputs="polling_day")
def announce_election(node):
    """
    The authority node announces the initial polling day immediately after
    initialisation. The polling day packet is automatically sent out to the
    three separate destination nodes.
    """
    node.get_ros_node().get_logger().info("Let's vote!")
    return node.make_packet("polling_day")

@neuros_function(inputs=All("vote"), outputs="polling_day")
def polling_station(node, votes):
    """
    Vote packets must be received from all of the voter source nodes before
    this hook will be invoked. When invoked it is passed a list of votes, where
    each element is taken from a single source node. Finally, the authority
    waits for 5 seconds and then announces another polling day.
    """
    votes = [v.data for v in votes]
    vote_count = 1
    most_popular = "none"
    for candidate in set(votes):
        count = votes.count(candidate)
        if count > vote_count:
            vote_count = count
            most_popular = candidate
    node.get_ros_node().get_logger().info(f"The winner is '{most_popular}'")
    node.get_ros_node().get_logger().info("Let's vote!")
    return node.make_packet("polling_day")
