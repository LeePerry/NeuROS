#!/usr/bin/env python3
# Copyright (c) 2024 Lee Perry

"""
This metric measures the accuracy and consistency of the provided tick 
controller mechanisms.
"""

import common.data
import common.plot
import common.run

data_path = "/tmp/1_tick_interval.txt"

def create_data():
    """
    Generates the results data by running the 1_tick example for 5 minutes
    and writing the results to a file.
    """
    data = common.data.Writer(data_path)
    common.run.process_for(
        ["./launch.py", "--project", "./examples/1_tick/clock/clock.json"],
        5 * 60,
        data.write)

def process_data():
    """
    Parses and analysis the tick intervals from the results data file.

    Produces mean, standard deviation, percentages within bounds and an 
    interval histogram.
    """
    data = common.data.Reader(data_path)
    parser = common.data.Parser("\[INFO\] \[(\d*\.?\d+)\] \[clock\]: T.*")
    data.read(parser.parse)
    # DISCARDS THE ANOMOLOUR FIRST INTERVAL AT STARTUP
    intervals = parser.intervals()[1:]
    common.data.basic_stats(intervals)
    common.data.percentage_within_x_of_target(intervals, 1.0, 0.010)
    common.data.percentage_within_x_of_target(intervals, 1.0, 0.005)
    common.data.percentage_within_x_of_target(intervals, 1.0, 0.002)
    common.data.percentage_within_x_of_target(intervals, 1.0, 0.001)
    common.plot.histogram(intervals,
                          "Interval (Target 1.0)",
                          relative_frequency=False,
                          bins=20)

if __name__ == '__main__':
    create_data()
    process_data()
