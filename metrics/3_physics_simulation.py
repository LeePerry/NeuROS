#!/usr/bin/env python3
# Copyright (c) 2024 Lee Perry

import common.data
import common.plot
import common.run

import numpy as np
import time

"""
This metric measures the sim to real time ratio of the physics simulation.
"""

data_paths = ["1_elevator_standard",
              "2_elevator_no_discard_limit",
              "3_elevator_asynchronous_gazebo"]

def create_data():
    """
    Generates the results data by running the 3_physics_simulation example for 
    5 minutes and writing the results to a file.
    """
    for name in data_paths:
        data = common.data.Writer(f"log/3_physics_simulation_{name}.txt")
        common.run.process_for(
            ["./launch.py", "--monitor-system-load", "--project", 
                f"./examples/3_physics_simulation/elevator/{name}.json"],
            5 * 60,
            data.write)
        print("==== WAITING ====")
        time.sleep(60 * 2)

def process_data():
    """
    Parses and analysis the simulated and real time from the results data file.

    Produces: Stats for Sim and real time intervals, real vs. sim comparison
    and CPU time series.
    """
    for name in data_paths:
        data = common.data.Reader(f"log/3_physics_simulation_{name}.txt")
        real_time_parser = common.data.Parser("\[INFO\] \[(\d*\.?\d+)\] \[physics\]: Simulated time: .*")
        data.read(real_time_parser.parse)

        print("==== Real Time ====")
        real_time_intervals = real_time_parser.intervals()
        common.data.basic_stats(real_time_intervals)
        binwidth = 0.05
        common.plot.histogram(f"log/3_physics_simulation_{name}_real_time_interval_histogram.png",
                            real_time_intervals,
                            "Interval (seconds)",
                            relative_frequency=False,
                            bins=np.arange(min(real_time_intervals), max(real_time_intervals) + binwidth, binwidth), 
                            xlimits=[2.75, 5.0],
                            ylimits=[0, 26])

        data = common.data.Reader(f"log/3_physics_simulation_{name}.txt")
        sim_time_parser = common.data.Parser("\[INFO\] \[.*\] \[physics\]: Simulated time: (\d*\.?\d+)")
        data.read(sim_time_parser.parse)

        print("==== Simulated Time ====")
        sim_time_intervals = sim_time_parser.intervals()
        common.data.basic_stats(sim_time_intervals)
        binwidth = 0.00025
        common.plot.histogram(f"log/3_physics_simulation_{name}_simulated_time_interval_histogram.png",
                            sim_time_intervals,
                            "Interval (seconds)",
                            relative_frequency=False,
                            bins=np.arange(min(sim_time_intervals), max(sim_time_intervals) + binwidth, binwidth),
                            xlimits=[1.0, 1.0035],
                            ylimits=[0, 80])

        if len(real_time_parser.samples()) != len(sim_time_parser.samples()):
            raise Exception("Different number of sim and real time samples!")

        print("==== Real vs. Simulated Time Summary ====")
        real_time_samples = real_time_parser.samples()
        real_time_samples = [s - real_time_samples[0] for s in real_time_samples]
        sim_time_samples = sim_time_parser.samples()
        sim_time_samples = [s - sim_time_samples[0] for s in sim_time_samples]
        print(f"Average Time Ratio: {sim_time_samples[-1] / real_time_samples[-1]}")
        common.plot.line(f"log/3_physics_simulation_{name}_real_vs_simulated_time_series.png",
                        real_time_samples,
                        sim_time_samples,
                        "Real Time (seconds)",
                        "Simulated Time (seconds)")

        print("==== Physics Sim CPU Series ====")
        common.plot.all_cpu_time_series(f"log/3_physics_simulation_{name}.txt",
                                        f"log/3_physics_simulation_{name}_cpu_time_series.png")

if __name__ == '__main__':
    #create_data()
    process_data()
