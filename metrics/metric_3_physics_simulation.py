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

data_paths = { "1_elevator_standard"            : "Strict Synchronisation",
               "2_elevator_no_discard_limit"    : "No Discard Limit",
               "3_elevator_asynchronous_gazebo" : "Asynchronous Gazebo" }

def create_data():
    """
    Generates the results data by running the 3_physics_simulation example for 
    5 minutes and writing the results to a file.
    """
    for name in data_paths.keys():
        print("==== STARTING NEW EXPERIMENT ====")
        data = common.data.Writer(f"results_data/3_physics_simulation_{name}.txt")
        common.run.process_for(
            ["./launch.py", "--monitor-system-load", "--project", 
                f"./examples/3_physics_simulation/elevator/{name}.json"],
            5 * 60,
            data.write)
        print("==== WAITING (PLEASE CHECK ALL PROCESSES ARE STOPPED) ====")
        time.sleep(60 * 2)

def process_data():
    """
    Parses and analysis the simulated and real time from the results data file.

    Produces: Stats for Sim and real time intervals, real vs. sim comparison
    and CPU time series.
    """
    real_sim_comparison = {}
    sent_received_comparison = {}
    
    for name, alias in data_paths.items():

        print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
        print(alias)
        print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
        data = common.data.Reader(f"results_data/3_physics_simulation_{name}.txt")

        print("==== Real Time ====")
        real_time_parser = common.data.Parser("\[INFO\] \[(\d*\.?\d+)\] \[physics\]: Simulated time: .*")
        data.read(real_time_parser.parse)
        real_time_intervals = real_time_parser.intervals()
        common.data.basic_stats(real_time_intervals)
        binwidth = 0.05
        common.plot.histogram(f"results_data/3_physics_simulation_{name}_real_time_interval_histogram.png",
                            real_time_intervals,
                            "Interval (seconds)",
                            relative_frequency=False,
                            bins=np.arange(min(real_time_intervals), max(real_time_intervals) + binwidth, binwidth), 
                            xlimits=[2.5, 6.5],
                            ylimits=[0, 20])

        print("==== Simulated Time ====")
        sim_time_parser = common.data.Parser("\[INFO\] \[.*\] \[physics\]: Simulated time: (\d*\.?\d+)")
        data.read(sim_time_parser.parse)
        sim_time_intervals = sim_time_parser.intervals()
        common.data.basic_stats(sim_time_intervals)
        binwidth = 0.00025
        common.plot.histogram(f"results_data/3_physics_simulation_{name}_simulated_time_interval_histogram.png",
                            sim_time_intervals,
                            "Interval (seconds)",
                            relative_frequency=False,
                            bins=np.arange(min(sim_time_intervals), max(sim_time_intervals) + binwidth, binwidth),
                            xlimits=[1.0, 1.0035],
                            ylimits=[0, 80])
        
        print("==== Real vs. Simulated Time ====")
        if len(real_time_parser.samples()) != len(sim_time_parser.samples()):
            raise Exception("Different number of sim and real time samples!")
        real_time_samples = real_time_parser.samples()
        real_time_offsets = [s - real_time_samples[0] for s in real_time_samples]
        sim_time_samples = sim_time_parser.samples()
        sim_time_samples = [s - sim_time_samples[0] for s in sim_time_samples]
        print(f"Average Time Ratio: {sim_time_samples[-1] / real_time_offsets[-1]}")
        common.plot.line(f"results_data/3_physics_simulation_{name}_real_vs_simulated_time_series.png",
                        real_time_offsets,
                        sim_time_samples,
                        "Real Time (seconds)",
                        "Simulated Time (seconds)")
        real_sim_comparison[alias] = (real_time_offsets, sim_time_samples)

        print("==== Dropped Packets Gazebo -> NeuROS ====")
        level_parser = common.data.Parser("\[INFO\] \[(\d*\.?\d+)\] \[physics\]: Level: .*")
        data.read(level_parser.parse)
        level_samples = level_parser.samples()
        groups = {}
        group_index = 1
        member_index = 0
        while group_index < len(real_time_samples):
            current_group = []
            while (member_index < len(level_samples) and 
                    level_samples[member_index] <= real_time_samples[group_index]):
                current_group.append(level_samples[member_index])
                member_index += 1
            groups[real_time_samples[group_index]] = current_group
            if member_index == len(level_samples):
                break
            group_index += 1
        number_of_expected_packets = 5 * len(groups)
        number_of_actual_packets = sum(len(v) for _, v in groups.items())
        number_of_dropped_packets = number_of_expected_packets - number_of_actual_packets
        print(f"Level: {number_of_dropped_packets * 100 / number_of_expected_packets}%")

        print("==== Dropped Packets NeuROS -> NeuROS ====")
        sent_received_comparison[alias] = common.data.dropped_packet_summary(data)

        print("==== CPU ====")
        labels, datasets = common.plot.all_cpu_time_series(f"results_data/3_physics_simulation_{name}.txt",
                                                           f"results_data/3_physics_simulation_{name}_cpu_time_series.png")
        for label, data in zip(labels, datasets):
            print(f"____ {label} ____")
            common.data.basic_stats(data)
        
        print("==== Memory Consumption ====")
        common.plot.memory_consumption_time_series(f"results_data/3_physics_simulation_{name}.txt",
                                                   f"results_data/3_physics_simulation_{name}_memory_percent_time_series.png")
        labels, datasets = common.plot.memory_consumption_time_series(f"results_data/3_physics_simulation_{name}.txt",
                                                                      f"results_data/3_physics_simulation_{name}_memory_absolute_time_series.png",
                                                                      percent=False)
        for label, data in zip(labels, datasets):
            print(f"____ {label} ____")
            common.data.basic_stats(data)

        print("==== Network ====")
        labels, datasets = common.plot.network_speeds_time_series(f"results_data/3_physics_simulation_{name}.txt",
                                                                  f"results_data/3_physics_simulation_{name}_network_speeds_time_series.png")
        for label, data in zip(labels, datasets):
            print(f"____ {label} ____")
            common.data.basic_stats(data)

    print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
    print("Combined Graphs")
    print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
    
    print("==== Combined Real vs. Simulated Time ====")
    all_labels = real_sim_comparison.keys()
    all_x = [data[0] for _, data in real_sim_comparison.items()]
    all_y = [data[1] for _, data in real_sim_comparison.items()]
    common.plot.multi_line(f"results_data/3_physics_simulation_combined_real_vs_simulated_time_series.png",
                           all_labels,
                           all_x,
                           all_y,
                           "Real Time (seconds)",
                           "Simulated Time (seconds)")
    
    print("==== Combined Sent / Received Packets ====")
    common.plot.sent_received_packets(
        sent_received_comparison,
        "results_data/3_physics_simulation_combined_packet_counts.png")
    
    print("==== Memory Consumption ====")
    common.plot.memory_consumption_time_series(
        [f"results_data/3_physics_simulation_{name}.txt" for name in data_paths.keys()],
        "results_data/3_physics_simulation_combined_memory_percent_time_series.png",
        labels=[l for _, l in data_paths.items()])
    common.plot.memory_consumption_time_series(
        [f"results_data/3_physics_simulation_{name}.txt" for name in data_paths.keys()],
        "results_data/3_physics_simulation_combined_memory_absolute_time_series.png",
        labels=[l for _, l in data_paths.items()],
        percent=False)

if __name__ == '__main__':
    create_data()
    process_data()
