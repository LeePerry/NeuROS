#!/usr/bin/env python3
# Copyright (c) 2024 Lee Perry

"""
This metric measures the accuracy and consistency of the provided tick 
controller mechanisms.
"""

import numpy as np

import common.data
import common.plot
import common.run

data_paths = { "1_tick_interval"          : "NeuROS",
               "1_tick_raw_ros2_interval" : "Pure ROS2" }

def create_data():
    """
    Generates the results data by running the 1_tick example for 5 minutes
    and writing the results to a file.
    """
    data = common.data.Writer(data_path)
    common.run.process_for(
        ["./launch.py", "--monitor-system-load", "--project", 
            "./examples/1_tick/clock/clock.json"],
        5 * 60,
        data.write)

def process_data():
    """
    Parses and analysis the tick intervals from the results data file.

    Produces: Mean, standard deviation, percentages within bounds and an 
    interval histogram. Also CPU time series.
    """
    for data_path, alias in data_paths.items():
        
        print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
        print(alias)
        print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")

        data = common.data.Reader(f"results_data/{data_path}.txt")

        print("==== Interval ====")
        parser = common.data.Parser("\[INFO\] \[(\d*\.?\d+)\] \[clock\]: T.*")
        data.read(parser.parse)
        intervals = parser.intervals()[1:]
        common.data.basic_stats(intervals)
        common.data.percentage_within_x_of_target(intervals, 1.0, 0.010)
        common.data.percentage_within_x_of_target(intervals, 1.0, 0.005)
        common.data.percentage_within_x_of_target(intervals, 1.0, 0.002)
        common.data.percentage_within_x_of_target(intervals, 1.0, 0.001)
        binwidth = 0.0005
        common.plot.histogram(f"results_data/{data_path}_histogram.png",
                                intervals,
                                "Interval (Target 1.0)",
                                relative_frequency=False,
                                bins=np.arange(min(intervals), max(intervals) + binwidth, binwidth), 
                                xlimits=[0.99, 1.01],
                                ylimits=[0, 120])
        
        if alias == "Pure ROS2":
            continue

        print("==== CPU ====")
        common.plot.all_cpu_time_series(f"results_data/{data_path}.txt", 
                                        f"results_data/{data_path}_cpu_time_series.png")
        
        print("==== Memory Consumption ====")
        common.plot.memory_consumption_time_series(f"results_data/{data_path}.txt",
                                                   f"results_data/{data_path}_memory_percent_time_series.png")
        common.plot.memory_consumption_time_series(f"results_data/{data_path}.txt",
                                                   f"results_data/{data_path}_memory_absolute_time_series.png",
                                                   percent=False)
        
        print("==== Network ====")
        common.plot.network_speeds_time_series(f"results_data/{data_path}.txt",
                                               f"results_data/{data_path}_network_speeds_time_series.png")

if __name__ == '__main__':
    create_data()
    process_data()
