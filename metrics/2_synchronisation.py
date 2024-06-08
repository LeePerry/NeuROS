#!/usr/bin/env python3
# Copyright (c) 2024 Lee Perry

import common.data
import common.plot
import common.run

"""
This metric confirms the ability of NeuROS to execute node user plugins 
concurrently.
"""

data_path = "/tmp/2_synchronisation.txt"

def create_data():
    """
    Generates the results data by running the 2_synchronisation/voting example 
    for 5 minutes and writing the results to a file.
    """
    data = common.data.Writer(data_path)
    common.run.process_for(
        ["./launch.py", "--project", "./examples/2_synchronisation/voting/election.json"],
        5 * 60,
        data.write)

def process_data():
    """
    Parses and analysis the concurrent execution time from the results data 
    file.

    Produces mean, standard deviation and execution time histogram.
    """
    data = common.data.Reader(data_path)
    parser = common.data.Parser("\[INFO\] \[(\d*\.?\d+)\] \[authority\]: Let's vote!")
    data.read(parser.parse)
    intervals = parser.intervals()
    common.data.basic_stats(intervals)
    common.plot.histogram(intervals,
                          "Execution Time (Target 5.0)",
                          relative_frequency=False,
                          bins=20)

if __name__ == '__main__':
    create_data()
    process_data()
