#!/usr/bin/env python3
# Copyright (c) 2024 Lee Perry

import common.data
import common.plot
import common.run

"""
This metric measures the sim to real time ratio of the physics simulation.
"""

data_path = "/tmp/3_physics_simulation.txt"

def create_data():
    """
    Generates the results data by running the 3_physics_simulation example for 
    5 minutes and writing the results to a file.
    """
    data = common.data.Writer(data_path)
    common.run.process_for(
        ["./launch.py", "--project", "./examples/3_physics_simulation/elevator/elevator.json"],
        5 * 60,
        data.write)

def process_data():
    """
    Parses and analysis the simulated and real time from the results data file.
    """
    data = common.data.Reader(data_path)
    real_time_parser = common.data.Parser("\[INFO\] \[(\d*\.?\d+)\] \[physics\]: Simulated time: .*")
    data.read(real_time_parser.parse)

    real_time_intervals = real_time_parser.intervals()
    print("================")
    print("Real Time")
    common.data.basic_stats(real_time_intervals)
    common.plot.histogram(real_time_intervals,
                          "Interval (seconds)",
                          relative_frequency=False,
                          bins=20)

    data = common.data.Reader(data_path)
    sim_time_parser = common.data.Parser("\[INFO\] \[.*\] \[physics\]: Simulated time: (\d*\.?\d+)")
    data.read(sim_time_parser.parse)

    sim_time_intervals = sim_time_parser.intervals()
    print("================")
    print("Simulated Time:")
    common.data.basic_stats(sim_time_intervals)
    common.plot.histogram(sim_time_intervals,
                          "Interval (seconds)",
                          relative_frequency=False,
                          bins=20)

    if len(real_time_parser.samples()) != len(sim_time_parser.samples()):
        raise Exception("Different number of sim and real time samples!")

    real_time_samples = real_time_parser.samples()
    real_time_samples = [s - real_time_samples[0] for s in real_time_samples]
    sim_time_samples = sim_time_parser.samples()
    sim_time_samples = [s - sim_time_samples[0] for s in sim_time_samples]
    print("================")
    print(f"Average Time Ratio: {sim_time_samples[-1] / real_time_samples[-1]}")
    common.plot.line(real_time_samples,
                     sim_time_samples,
                     "Real Time (seconds)",
                     "Simulated Time (seconds)")

if __name__ == '__main__':
    create_data()
    process_data()
