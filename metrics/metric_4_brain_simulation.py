#!/usr/bin/env python3
# Copyright (c) 2024 Lee Perry

import common.data
import common.plot
import common.run

"""
This metric measures the system load during physics and brain simulation.
"""

data_path = "results_data/4_brain_simulation.txt"

def create_data():
    """
    Generates the results data by running the 4_brain_simulation prediction example for 
    5 minutes and writing the results to a file.
    """
    data = common.data.Writer(data_path)
    common.run.process_for(
        ["./launch.py", "--monitor-system-load", "--project", 
            "examples/4_brain_simulation/whiskeye/2_prediction.json"],
        5 * 60,
        data.write)

def process_data():
    """
    Parses and analysis the system load results data file.
    """
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
    common.plot.line(f"results_data/4_brain_simulation_real_vs_simulated_time_series.png",
                    real_time_offsets,
                    sim_time_samples,
                    "Real Time (seconds)",
                    "Simulated Time (seconds)")

    print("==== Dropped Packets NeuROS -> NeuROS ====")
    combined = { "Complete Neurorobotics Experiment" :
        common.data.dropped_packet_summary(data, aliases={"odom_correction" : "head_dir_prediction"})}
    common.plot.sent_received_packets(combined, "results_data/4_brain_simulation_packet_counts.png")

    print("==== CPU ====")
    labels, datasets = common.plot.all_cpu_time_series(f"results_data/4_brain_simulation.txt",
                                                       f"results_data/4_brain_simulation_cpu_time_series.png")
    for label, data in zip(labels, datasets):
        print(f"____ {label} ____")
        common.data.basic_stats(data)

    print("==== Memory Consumption ====")
    common.plot.memory_consumption_time_series(f"results_data/4_brain_simulation.txt",
                                               f"results_data/4_brain_simulation_memory_percent_time_series.png")
    labels, datasets = common.plot.memory_consumption_time_series(f"results_data/4_brain_simulation.txt",
                                                                  f"results_data/4_brain_simulation_memory_absolute_time_series.png",
                                                                  percent=False)
    for label, data in zip(labels, datasets):
        print(f"____ {label} ____")
        common.data.basic_stats(data)

    print("==== Network ====")
    labels, datasets = common.plot.network_speeds_time_series(f"results_data/4_brain_simulation.txt",
                                                              f"results_data/4_brain_simulation_network_speeds_time_series.png")
    for label, data in zip(labels, datasets):
        print(f"____ {label} ____")
        common.data.basic_stats(data)

if __name__ == '__main__':
    create_data()
    process_data()
