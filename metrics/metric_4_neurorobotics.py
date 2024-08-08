#!/usr/bin/env python3
# Copyright (c) 2024 Lee Perry

import common.data
import common.plot
import common.run

import time

"""
This metric measures the system load during physics and brain simulation.
"""

data_paths = { "2_prediction_1_tick_controlled_20ms"  : "Tick Controlled (20ms Interval)",
               "2_prediction_1_tick_controlled_60ms"  : "Tick Controlled (60ms Interval)",
               "2_prediction_1_tick_controlled_100ms" : "Tick Controlled (100ms Interval)",
               "2_prediction_2_strict"                : "Strict Synchronisation",
               "2_prediction_3_no_discard_limit"      : "No Discard Limit",
               "2_prediction_4_asynchronous"          : "Asynchronous Gazebo" }

def create_data():
    """
    Generates the results data by running the 4_neurorobotics prediction example for
    5 minutes and writing the results to a file.
    """
    for name in data_paths.keys():
        print(f"==== STARTING {name} ====")
        data = common.data.Writer(f"results_data/4_neurorobotics_{name}.txt")
        common.run.process_for(
            ["./launch.py", "--monitor-system-load", "--project",
                f"examples/4_neurorobotics/whiskeye/{name}.json"],
            5 * 60,
            data.write)
        print("==== WAITING (PLEASE CHECK ALL PROCESSES ARE STOPPED) ====")
        time.sleep(10)

def process_data():
    """
    Parses and analysis the system load results data file.
    """
    for name, alias in data_paths.items():

        print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
        print(alias)
        print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
        data_path = f"results_data/4_neurorobotics_{name}.txt"
        data = common.data.Reader(data_path)

        print("==== Real vs. Simulated Time ====")
        real_time_parser = common.data.Parser("\[INFO\] \[(\d*\.?\d+)\] \[robot\]: Simulated time: .*")
        data.read(real_time_parser.parse)
        sim_time_parser = common.data.Parser("\[INFO\] \[.*\] \[robot\]: Simulated time: (\d*\.?\d+)")
        data.read(sim_time_parser.parse)
        real_time_samples = real_time_parser.samples()
        real_time_offsets = [s - real_time_samples[0] for s in real_time_samples]
        sim_time_samples = sim_time_parser.samples()
        sim_time_samples = [s - sim_time_samples[0] for s in sim_time_samples]
        print(f"Average Time Ratio: {sim_time_samples[-1] / real_time_offsets[-1]}")
        common.plot.line(f"results_data/4_neurorobotics_{name}_real_vs_simulated_time_series.png",
                        real_time_offsets,
                        sim_time_samples,
                        "Real Time (seconds)",
                        "Simulated Time (seconds)")

        print("==== Dropped Packets NeuROS -> NeuROS ====")
        combined = { "Complete Neurorobotics Experiment" :
            common.data.dropped_packet_summary(data, aliases={"odom_correction" : "head_dir_prediction"})}
        common.plot.sent_received_packets(combined, f"results_data/4_neurorobotics_{name}_packet_counts.png")

        print("==== CPU ====")
        labels, datasets = common.plot.all_cpu_time_series(
            data_path, f"results_data/4_neurorobotics_{name}_cpu_time_series.png")
        for label, data in zip(labels, datasets):
            print(f"____ {label} ____")
            common.data.basic_stats(data)

        print("==== Memory Consumption ====")
        common.plot.memory_consumption_time_series(
            data_path, f"results_data/4_neurorobotics_{name}_memory_percent_time_series.png")
        labels, datasets = common.plot.memory_consumption_time_series(
            data_path, f"results_data/4_neurorobotics_{name}_memory_absolute_time_series.png", percent=False)
        for label, data in zip(labels, datasets):
            print(f"____ {label} ____")
            common.data.basic_stats(data)

        print("==== Network ====")
        labels, datasets = common.plot.network_speeds_time_series(
            data_path, f"results_data/4_neurorobotics_{name}_network_speeds_time_series.png")
        for label, data in zip(labels, datasets):
            print(f"____ {label} ____")
            common.data.basic_stats(data)

if __name__ == '__main__':
    create_data()
    process_data()
