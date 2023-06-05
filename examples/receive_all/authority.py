# Copyright (c) 2023 Lee Perry

import time

from neuros.hooks import neuros_initialise, neuros_receive_all

@neuros_initialise
def announce_election(node):
    node.get_logger().info("Let's vote!")
    event = node.get_outgoing("polling_day")
    event.send(event.create_packet())

@neuros_receive_all("vote")
def polling_station(node):
    vote_count = 0
    most_popular_candidate = "none"
    all_votes = [v.data for v in node.get_incoming("vote").get_all()]
    for candidate in set(all_votes):
        count = all_votes.count(candidate)
        if count > vote_count:
            vote_count = count
            most_popular_candidate = candidate
    node.get_logger().info(f"Counted {all_votes}. The winner is '{most_popular_candidate}'")
    time.sleep(5)
    announce_election(node)
