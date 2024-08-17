#!/usr/bin/env python3
# Copyright (c) 2024 Lee Perry

import common.data
import common.plot
import common.run

import math
import time

"""
This metric measures the system load during physics and brain simulation.
"""

data_paths = { "2_prediction_5_mixed" : "Mixed Approach" }

def create_data():
    """
    Generates the results data by running the 4_neurorobotics prediction example for
    5 minutes and writing the results to a file.
    """
    for name in data_paths.keys():
        print(f"==== STARTING {name} ====")
        data = common.data.Writer(f"results_data/4_neurorobotics_{name}.txt")
        common.run.process_for_simulated_duration(
            ["./launch.py", "--monitor-system-load", "--project",
                f"examples/4_neurorobotics/whiskeye/{name}.json"],
            20,
            data.write)
        print("==== WAITING (PLEASE CHECK ALL PROCESSES ARE STOPPED) ====")
        time.sleep(10)

def process_data():
    """
    Parses and analysis the system load results data file.
    """

    D = common.plot.DIGIT

    for name, alias in data_paths.items():

        print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
        print(alias)
        print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
        data_path = f"results_data/4_neurorobotics_{name}.txt"
        data = common.data.Reader(data_path)

        print("==== Head Direction Prediction ====")
        prefix = "\[INFO\] \[.*\] \[robot\]: Ground Truth: \[geometry_msgs.msg.Quaternion"
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
        prefix = "\[INFO\] \[.*\] \[snn\]: Spiking Neural Network: "
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
            xlabel="Simulated Time (seconds)", ylabel="Head Direction (rad)")

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
        print(f"Average Time Ratio: {speed}")
        common.plot.line(f"results_data/4_neurorobotics_{name}_real_vs_simulated_time_series.png",
                        real_time_offsets,
                        sim_time_samples,
                        "Real Time (seconds)",
                        "Simulated Time (seconds)")

        print("==== Dropped Packets Gazebo -> NeuROS ====")
        gt_parser = common.data.Parser("\[INFO\] \[(\d*\.?\d+)\] \[robot\]: Received _ground_truth")
        data.read(gt_parser.parse)
        expected_gt_count = 1001
        received_gt_count = len(gt_parser.samples())
        dropped_gt_count = expected_gt_count - received_gt_count
        print(f"Ground Truth: {dropped_gt_count * 100 / received_gt_count}%")
        imu_parser = common.data.Parser("\[INFO\] \[(\d*\.?\d+)\] \[robot\]: Received _imu")
        data.read(imu_parser.parse)
        expected_imu_count = 1002
        received_imu_count = len(imu_parser.samples())
        dropped_imu_count = expected_imu_count - received_imu_count
        print(f"IMU: {dropped_imu_count * 100 / received_imu_count}%")

        print("==== Dropped Packets NeuROS -> NeuROS ====")
        pkt_loss = common.data.dropped_packet_summary(data, aliases={"odom_correction" : "head_dir_prediction"})
        common.plot.sent_received_packets(
            f"results_data/4_neurorobotics_{name}_packet_counts.png",
            { "Complete Neurorobotics Experiment" : pkt_loss})

        print("==== CPU ====")
        cpus = []
        labels, datasets = common.plot.all_cpu_time_series(
            data_path, f"results_data/4_neurorobotics_{name}_cpu_time_series.png",
            range_stop=3 * 60)
        for label, data in zip(labels, datasets):
            print(f"____ {label} ____")
            cpus.append(common.data.basic_stats(data)[0])

        print("==== Memory Consumption ====")
        common.plot.memory_consumption_time_series(
            data_path, f"results_data/4_neurorobotics_{name}_memory_percent_time_series.png")
        memory = []
        labels, datasets = common.plot.memory_consumption_time_series(
            data_path, f"results_data/4_neurorobotics_{name}_memory_absolute_time_series.png", percent=False)
        for label, data in zip(labels, datasets):
            print(f"____ {label} ____")
            memory.append(common.data.basic_stats(data)[0])

        print("==== Network ====")
        labels, datasets = common.plot.network_speeds_time_series(
            data_path, f"results_data/4_neurorobotics_{name}_network_speeds_time_series.png")
        for label, data in zip(labels, datasets):
            print(f"____ {label} ____")
            common.data.basic_stats(data)

if __name__ == '__main__':
    create_data()
    process_data()
