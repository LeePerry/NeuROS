# Copyright (c) 2024 Lee Perry

import matplotlib.pyplot as plt
import numpy as np

import common.data

main_colour = "teal"

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
                weights=np.zeros_like(data) + 100.0 / data.size, 
                color=main_colour)
        ax.set_ylabel('Frequency (%)', size=12)
    else:
        ax.hist(data, bins=bins, color=main_colour)
        ax.set_ylabel('Count', size=12)
    ax.set_xlabel(x_axis, size=12)
    fig.tight_layout()
    plt.savefig(path)

def line(path, x, y, x_axis, y_axis):
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.plot(x, y)
    ax.set_xlabel(x_axis, size=12)
    ax.set_ylabel(y_axis, size=12)
    fig.tight_layout()
    plt.savefig(path)

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

def multi_time_series(path, datasets, labels, y_axis, y_lim=[]):
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
    fig.tight_layout()
    plt.savefig(path)

def all_cpu_time_series(from_path, to_path, range_start=0, range_stop=-1):
    datasets = []
    labels = []
    for core in range(1, 9):
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
