#!/usr/bin/env python3
# Copyright (c) 2024 Lee Perry

import common.data
import common.plot
import common.run

import numpy as np

import math
import statistics
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

def process_nrp_data():
    combined = { common.plot.HEAD_DIRECTION_ESTIMATE  : {},
                 common.plot.DROPPED_PACKETS          : {},
                 common.plot.DELIVERED_PACKET_PERCENT   : {},
                 common.plot.CPU_MEAN                 : {},
                 common.plot.MEMORY_MEAN              : {},
                 common.plot.SIM_TIME_PERCENT_OF_REAL : {} }

    alias = "Neurorobotics Platform (NRP)"
    print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
    print(alias)
    print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
    system_load_data_path = "results_data/nrp/system_load.txt"

    print("==== Head Direction Prediction ====")
    data = common.data.Reader("results_data/nrp/all_spikes.csv")
    id_parser = common.data.Parser(f"({common.plot.DIGIT}),{common.plot.DIGIT}")
    data.read(id_parser.parse)
    t_parser = common.data.Parser(f"{common.plot.DIGIT},({common.plot.DIGIT})")
    data.read(t_parser.parse)
    t_bucket_width = 500
    t = t_parser.samples()
    t_bucket_high = t[0] + t_bucket_width
    t_bucket_centre = t[0] + (t_bucket_width / 2)
    t_bucket = []
    x = []
    y = []
    for id, t in zip(id_parser.samples(), t):
        if t < t_bucket_high:
            t_bucket.append(id)
        else:
            if t_bucket:
                print(f"Adding {t_bucket}")
                spike = statistics.mode(t_bucket)
                x.append(((spike - 90) / 90) * math.pi)
                y.append(t_bucket_centre / 1000)
                t_bucket = []
            t_bucket_high += t_bucket_width
            t_bucket_centre += t_bucket_width
    common.plot.head_direction(
            f"results_data/4_nrp_head_direction_prediction.png",
            { alias : (x, y)}, # "Ground Truth" : (gt_x, gt_y),
            xlabel="Real Time (seconds)", ylabel="Head Direction (rad)")

    print("==== Real vs. Simulated Time ====")
    data = common.data.Reader("results_data/nrp/time_stats.txt")
    real_time_parser = common.data.Parser("\[INFO\] \[(\d*\.?\d+)\] \[nrp-sim-real-time-logger\]: Simulated time: .*")
    data.read(real_time_parser.parse)
    sim_time_parser = common.data.Parser("\[INFO\] \[.*\] \[nrp-sim-real-time-logger\]: Simulated time: (\d*\.?\d+)")
    data.read(sim_time_parser.parse)
    real_time_samples = real_time_parser.samples()
    real_time_offsets = [s - real_time_samples[0] for s in real_time_samples]
    sim_time_samples = sim_time_parser.samples()
    sim_time_samples = [s - sim_time_samples[0] for s in sim_time_samples]
    speed = sim_time_samples[-1] / real_time_offsets[-1]
    combined[common.plot.SIM_TIME_PERCENT_OF_REAL][alias] = speed * 100
    print(f"Average Time Ratio: {speed}")

    print("==== Dropped Packets NeuROS -> NeuROS ====")
    # TODO NEED TO VERIFY THIS!
    combined[common.plot.DELIVERED_PACKET_PERCENT][alias] = 100

    print("==== CPU ====")
    cpus = []
    labels, datasets = common.plot.all_cpu_time_series(
        system_load_data_path, f"results_data/4_nrp_cpu_time_series.png")
    for label, data in zip(labels, datasets):
        print(f"____ {label} ____")
        cpus.append(common.data.basic_stats(data)[0])
    combined[common.plot.CPU_MEAN][alias] = np.mean(cpus)

    print("==== Memory Consumption ====")
    common.plot.memory_consumption_time_series(
        system_load_data_path, f"results_data/4_nrp_memory_percent_time_series.png")
    memory = []
    labels, datasets = common.plot.memory_consumption_time_series(
        system_load_data_path, f"results_data/4_nrp_memory_absolute_time_series.png", percent=False)
    for label, data in zip(labels, datasets):
        print(f"____ {label} ____")
        memory.append(common.data.basic_stats(data)[0])
    combined[common.plot.MEMORY_MEAN][alias] = np.mean(memory)

    return combined

def process_data():
    """
    Parses and analysis the system load results data file.
    """

    D = common.plot.DIGIT
    combined = process_nrp_data() # init to nrp baseline

    for name, alias in data_paths.items():

        print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
        print(alias)
        print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
        data_path = f"results_data/4_neurorobotics_{name}.txt"
        data = common.data.Reader(data_path)

        print("==== Head Direction Prediction ====")
        prefix = "\[INFO\] \[.*\] \[visualiser\]: Ground Truth: \[geometry_msgs.msg.Quaternion"
        gt_z_parser = common.data.Parser(f"{prefix}\(x=0.0, y=0.0, z=({D}), w={D}\), {D}\]")
        data.read(gt_z_parser.parse)
        gt_w_parser = common.data.Parser(f"{prefix}\(x=0.0, y=0.0, z={D}, w=({D})\), {D}\]")
        data.read(gt_w_parser.parse)
        gt_t_parser = common.data.Parser(f"{prefix}\(x=0.0, y=0.0, z={D}, w={D}\), ({D})\]")
        data.read(gt_t_parser.parse)
        gt_x = []
        for z, w, t in zip(gt_z_parser.samples(), gt_w_parser.samples(), gt_t_parser.samples()):
            gt_x.append(math.atan2(2.0 * (w * z), 1.0 - 2.0 * (z * z)))
        gt_y = gt_t_parser.samples()
        gt_y = [s - gt_y[0] for s in gt_y]
        prefix = "\[INFO\] \[.*\] \[visualiser\]: Spiking Neural Network: "
        angle_parser = common.data.Parser(f"{prefix}\[({D}), {D}\]")
        data.read(angle_parser.parse)
        t_parser = common.data.Parser(f"{prefix}\[{D}, ({D})\]")
        data.read(t_parser.parse)
        x = angle_parser.samples()
        x = [(s / 180) * math.pi for s in x]
        y = t_parser.samples()
        y = [s - y[0] for s in y]
        common.plot.head_direction(
            f"results_data/4_neurorobotics_{name}_head_direction_prediction.png",
            {"Ground Truth" : (gt_x, gt_y), alias : (x, y)},
            xlabel="Real Time (seconds)", ylabel="Head Direction (rad)")

        print("==== Real vs. Simulated Time ====")
        real_time_parser = common.data.Parser("\[INFO\] \[(\d*\.?\d+)\] \[robot\]: Simulated time: .*")
        data.read(real_time_parser.parse)
        sim_time_parser = common.data.Parser("\[INFO\] \[.*\] \[robot\]: Simulated time: (\d*\.?\d+)")
        data.read(sim_time_parser.parse)
        real_time_samples = real_time_parser.samples()
        real_time_offsets = [s - real_time_samples[0] for s in real_time_samples]
        sim_time_samples = sim_time_parser.samples()
        sim_time_samples = [s - sim_time_samples[0] for s in sim_time_samples]
        speed = sim_time_samples[-1] / real_time_offsets[-1]
        combined[common.plot.SIM_TIME_PERCENT_OF_REAL][alias] = speed * 100
        print(f"Average Time Ratio: {speed}")
        common.plot.line(f"results_data/4_neurorobotics_{name}_real_vs_simulated_time_series.png",
                        real_time_offsets,
                        sim_time_samples,
                        "Real Time (seconds)",
                        "Simulated Time (seconds)")

        print("==== Dropped Packets NeuROS -> NeuROS ====")
        pkt_loss = common.data.dropped_packet_summary(data, aliases={"odom_correction" : "head_dir_prediction"})
        combined[common.plot.DROPPED_PACKETS][alias] = pkt_loss
        combined[common.plot.DELIVERED_PACKET_PERCENT][alias] = common.data.dropped_packet_percentage(pkt_loss)
        common.plot.sent_received_packets(
            { "Complete Neurorobotics Experiment" : pkt_loss},
            f"results_data/4_neurorobotics_{name}_packet_counts.png")

        print("==== CPU ====")
        cpus = []
        labels, datasets = common.plot.all_cpu_time_series(
            data_path, f"results_data/4_neurorobotics_{name}_cpu_time_series.png")
        for label, data in zip(labels, datasets):
            print(f"____ {label} ____")
            cpus.append(common.data.basic_stats(data)[0])
        combined[common.plot.CPU_MEAN][alias] = np.mean(cpus)

        print("==== Memory Consumption ====")
        common.plot.memory_consumption_time_series(
            data_path, f"results_data/4_neurorobotics_{name}_memory_percent_time_series.png")
        memory = []
        labels, datasets = common.plot.memory_consumption_time_series(
            data_path, f"results_data/4_neurorobotics_{name}_memory_absolute_time_series.png", percent=False)
        for label, data in zip(labels, datasets):
            print(f"____ {label} ____")
            memory.append(common.data.basic_stats(data)[0])
        combined[common.plot.MEMORY_MEAN][alias] = np.mean(memory)

        print("==== Network ====")
        labels, datasets = common.plot.network_speeds_time_series(
            data_path, f"results_data/4_neurorobotics_{name}_network_speeds_time_series.png")
        for label, data in zip(labels, datasets):
            print(f"____ {label} ____")
            common.data.basic_stats(data)

    common.plot.sent_received_packets(
        combined[common.plot.DROPPED_PACKETS],
        "results_data/4_neurorobotics_combined_packet_counts.png",
        xlabelrot=90)

    common.plot.horizontal_bar(
        "results_data/4_neurorobotics_combined_packet_delivered.png",
        combined[common.plot.DELIVERED_PACKET_PERCENT],
        xlabel="Packets Delivered (%)")

    common.plot.horizontal_bar(
        "results_data/4_neurorobotics_combined_cpu_utilisation.png",
        combined[common.plot.CPU_MEAN],
        xlabel="Mean CPU Utilisation (%)")

    common.plot.horizontal_bar(
        "results_data/4_neurorobotics_combined_memory_consumption.png",
        combined[common.plot.MEMORY_MEAN],
        xlabel="Mean Memory Consumption (GB)")

    common.plot.horizontal_bar(
        "results_data/4_neurorobotics_percent_of_real_time_speed.png",
        combined[common.plot.SIM_TIME_PERCENT_OF_REAL],
        xlabel="Sim Speed as Percentage of Real Time (%)")

    return combined

if __name__ == '__main__':
    create_data()
    process_data()
