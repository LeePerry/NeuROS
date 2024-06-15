#!/usr/bin/env python3
# Copyright (c) 2024 Lee Perry

import common.data
import common.plot
import common.run

"""
This metric measures the sim to real time ratio of the physics simulation.
"""

data_path = "log/3_physics_simulation.txt"

def create_data():
    """
    Generates the results data by running the 3_physics_simulation example for 
    5 minutes and writing the results to a file.
    """
    data = common.data.Writer(data_path)
    common.run.process_for(
        ["./launch.py", "--monitor-system-load", "--project", 
            "./examples/3_physics_simulation/elevator/elevator.json"],
        5 * 60,
        data.write)

def process_data():
    """
    Parses and analysis the simulated and real time from the results data file.

    Produces: Stats for Sim and real time intervals, real vs. sim comparison
    and CPU time series.
    """
    data = common.data.Reader(data_path)
    real_time_parser = common.data.Parser("\[INFO\] \[(\d*\.?\d+)\] \[physics\]: Simulated time: .*")
    data.read(real_time_parser.parse)

    print("==== Real Time ====")
    real_time_intervals = real_time_parser.intervals()
    common.data.basic_stats(real_time_intervals)
    common.plot.histogram("log/3_physics_simulation_real_time_interval_histogram.png",
                          real_time_intervals,
                          "Interval (seconds)",
                          relative_frequency=False,
                          bins=20)

    data = common.data.Reader(data_path)
    sim_time_parser = common.data.Parser("\[INFO\] \[.*\] \[physics\]: Simulated time: (\d*\.?\d+)")
    data.read(sim_time_parser.parse)

    print("==== Simulated Time ====")
    sim_time_intervals = sim_time_parser.intervals()
    common.data.basic_stats(sim_time_intervals)
    common.plot.histogram("log/3_physics_simulation_simulated_time_interval_histogram.png",
                          sim_time_intervals,
                          "Interval (seconds)",
                          relative_frequency=False,
                          bins=20)

    if len(real_time_parser.samples()) != len(sim_time_parser.samples()):
        raise Exception("Different number of sim and real time samples!")

    print("==== Real vs. Simulated Time Summary ====")
    real_time_samples = real_time_parser.samples()
    real_time_samples = [s - real_time_samples[0] for s in real_time_samples]
    sim_time_samples = sim_time_parser.samples()
    sim_time_samples = [s - sim_time_samples[0] for s in sim_time_samples]
    print(f"Average Time Ratio: {sim_time_samples[-1] / real_time_samples[-1]}")
    common.plot.line("log/3_physics_simulation_real_vs_simulated_time_series.png",
                     real_time_samples,
                     sim_time_samples,
                     "Real Time (seconds)",
                     "Simulated Time (seconds)")

    print("==== Physics Sim CPU Series ====")
    # JUST TAKING THE 2nd MINUTE OF DATA (OFTEN HIGH AT START)
    common.plot.all_cpu_time_series(data_path,
                                    "log/3_physics_simulation_cpu_time_series.png")

if __name__ == '__main__':
    create_data()
    process_data()
