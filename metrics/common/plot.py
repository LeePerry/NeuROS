# Copyright (c) 2024 Lee Perry

import matplotlib.pyplot as plt
import numpy as np

import common.data

#main_colour = "teal"
#plt.style.use('seaborn-v0_8-pastel')
plt.style.use('tableau-colorblind10')

def histogram(path, data, x_axis, relative_frequency=False, bins=20, xlimits=None, ylimits=None):
    fig = plt.figure()
    ax = fig.add_subplot(111)
    if xlimits:
        plt.xlim(xlimits)
    if ylimits:
        plt.ylim(ylimits)
    if relative_frequency:
        data = np.array(data)
        ax.hist(data, 
                bins=bins, 
                weights=np.zeros_like(data) + 100.0 / data.size)
        ax.set_ylabel('Frequency (%)', size=12)
    else:
        ax.hist(data, bins=bins)
        ax.set_ylabel('Count', size=12)
    ax.set_xlabel(x_axis, size=12)
    fig.tight_layout()
    plt.savefig(path)
    plt.close()

def line(path, x, y, x_axis, y_axis):
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.plot(x, y)
    ax.set_xlabel(x_axis, size=12)
    ax.set_ylabel(y_axis, size=12)
    fig.tight_layout()
    plt.savefig(path)
    plt.close()

def multi_line(path, all_labels, all_x, all_y, x_axis, y_axis):
    fig = plt.figure()
    ax = fig.add_subplot(111)
    for label, x, y in zip(all_labels, all_x, all_y):
        ax.plot(x, y, label=label)
    ax.legend()
    ax.set_xlabel(x_axis, size=12)
    ax.set_ylabel(y_axis, size=12)
    fig.tight_layout()
    plt.savefig(path)
    plt.close()

def multi_time_series(path, datasets, labels, y_axis, y_lim=[], 
                      y_log_scale=False):
    fig = plt.figure()
    ax = fig.add_subplot(111)
    for data, label in zip(datasets, labels):
        ax.plot(data, label=label)
    ax.set_xlabel("Time (seconds)", size=12)
    ax.set_ylabel(y_axis, size=12)
    if len(labels) > 1:
        ax.legend()
    if y_lim:
        plt.ylim(y_lim)
    if y_log_scale:
        ax.set_yscale('log')
    fig.tight_layout()
    plt.savefig(path)
    plt.close()

def all_cpu_time_series(from_path, to_path, range_start=0, range_stop=-1):
    datasets = []
    labels = []
    for core in range(1, 64):
        data = common.data.Reader(from_path)
        parser = common.data.Parser.for_cpu(core)
        data.read(parser.parse)
        samples = parser.samples()[range_start:range_stop]
        if len(samples) == 0:
            break
        datasets.append(samples)
        labels.append(f"Core {core}")
    common.plot.multi_time_series(
        to_path, datasets, labels, "CPU Usage (%)", [0, 100])
    return labels, datasets

def memory_consumption_time_series(from_paths, to_path, percent=True, 
                                   labels=["Default"]):
    if not isinstance(from_paths, list):
        from_paths = [from_paths]
    
    gigabytes = 1024 * 1024 * 1024
    all_data = []
    for from_path in from_paths:
        parser = common.data.Parser.for_memory_consumption(percent)
        data = common.data.Reader(from_path)
        data.read(parser.parse)
        samples = parser.samples()
        if len(samples) == 0:
            raise Exception("No memory consumption stats!")
        if not percent:
            samples = np.array(samples)
            samples /= gigabytes
        all_data.append(samples)

    if percent:
        common.plot.multi_time_series(
            to_path, all_data, labels, "Memory Consumption (% of available)",
            y_lim=[0, 100])
    else:
        common.plot.multi_time_series(
            to_path, all_data, labels, "Memory Consumption (GB)",
            y_lim=[0, 10])
    return labels, all_data

def network_speeds_time_series(from_path, to_path):
    data = common.data.Reader(from_path)

    parser = common.data.Parser.for_network_speed(True)
    data.read(parser.parse)
    send_speeds = np.array(parser.samples())
    send_speeds /= 1024

    parser = common.data.Parser.for_network_speed(False)
    data.read(parser.parse)
    receive_speeds = np.array(parser.samples())
    receive_speeds /= 1024
    
    labels = ["Sent", "Received"]
    datasets = [send_speeds, receive_speeds]
    common.plot.multi_time_series(
        to_path, [send_speeds, receive_speeds], labels,
        "Network Throughput (KB/s)", y_log_scale=True)
    return labels, datasets

def grouped_multi_bar(to_path, groups, values, xlabel=None, ylabel=None, ylim=[]):
    _, ax = plt.subplots(layout='constrained')
    x = np.arange(len(groups))
    width = 1 / (len(values.keys()) + 1)
    multiplier = 0
    for attribute, measurement in values.items():
        offset = width * multiplier
        rects = ax.bar(x + offset, measurement, width, label=attribute)
        ax.bar_label(rects, padding=3)
        multiplier += 1
    ax.set_xticks(x + (width*1.5), groups)
    if len(groups) > 1:
        ax.legend(loc="upper left", ncols=1)
    if ylabel:
        ax.set_ylabel(ylabel)
    if xlabel:
        ax.set_ylabel(xlabel)
    if ylim:
        ax.set_ylim(ylim)
    plt.savefig(to_path)
    plt.close()