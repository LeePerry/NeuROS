#!/usr/bin/env python3
# Copyright (c) 2024 Lee Perry

import common.data
import common.plot
import common.run

"""
This metric confirms the ability of NeuROS to execute node user plugins 
concurrently.
"""

data_path = "log/2_synchronisation.txt"

def create_data():
    """
    Generates the results data by running the 2_synchronisation/voting example 
    for 5 minutes and writing the results to a file.
    """
    data = common.data.Writer(data_path)
    common.run.process_for(
        ["./launch.py", "--monitor-system-load", "--project", 
            "./examples/2_synchronisation/voting/election.json"],
        5 * 60,
        data.write)

def process_data():
    """
    Parses and analysis the concurrent execution time from the results data 
    file.

    Produces: Mean, standard deviation and execution time histogram. Also CPU
    time series.
    """
    data = common.data.Reader(data_path)
    parser = common.data.Parser("\[INFO\] \[(\d*\.?\d+)\] \[authority\]: Let's vote!")
    data.read(parser.parse)
    intervals = parser.intervals()

    print("==== Voting Interval ====")
    common.data.basic_stats(intervals)
    common.plot.histogram("log/2_synchronisation_voting_interval_histogram.png",
                          intervals,
                          "Execution Time (3 Parallel 5s Tasks)",
                          relative_frequency=False,
                          bins=20)

    print("==== Voting CPU Series ====")
    common.plot.all_cpu_time_series(data_path,
                                    "log/2_synchronisation_voting_cpu_time_series.png")
    
    print("==== Memory Consumption Series ====")
    common.plot.memory_consumption_time_series(data_path,
                                               "log/2_synchronisation_voting_memory_percent_time_series.png")
    common.plot.memory_consumption_time_series(data_path,
                                               "log/2_synchronisation_voting_memory_absolute_time_series.png",
                                               percent=False)

    print("==== Tick Network Series ====")
    common.plot.network_speeds_time_series(data_path,
                                           "log/2_synchronisation_voting_network_speeds_time_series.png")

if __name__ == '__main__':
    #create_data()
    process_data()
