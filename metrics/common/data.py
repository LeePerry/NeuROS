# Copyright (c) 2024 Lee Perry

import re
import numpy as np

def basic_stats(data):
    mean = np.mean(data)
    std = np.std(data)
    print(f"Count: {len(data)}")
    print(f"Mean: {mean}")
    print(f"Standard Deviation: {std}")
    return mean, std

def percentage_within_x_of_target(data, target, x):
    count = 0
    for i in data:
        if i >= (target - x) and i <= (target + x):
            count += 1
    print(f"Percentage within {x} of target: {count * 100 / len(data)}")

class Writer:

    def __init__(self, path):
        self._file = open(path, 'w')

    def write(self, line):
        self._file.write(line)

class Reader:

    def __init__(self, path):
        self._path = path

    def read(self, parser):
        with open(self._path) as f:
            for line in f.readlines():
                parser(line)

class Parser:

    @classmethod
    def for_cpu(cls, cpu):
        return cls("\[INFO\] \[.*\] \[system-load-monitor\]: CPU: \[" +
                   ("\d*\.?\d+, " * (cpu - 1)) +
                   "(\d*\.?\d+).*")

    @classmethod
    def for_network_speed(cls, send=False):
        return cls("\[INFO\] \[.*\] \[system-load-monitor\]: Network: " +
                   ("\[\d+, (\d+)\]" if send else "\[(\d+), \d+\]"))

    @classmethod
    def for_memory_consumption(cls, percent=True):
        return cls("\[INFO\] \[.*\] \[system-load-monitor\]: Memory: " +
                   ("\[\d+, (\d*\.?\d+)\]" if percent else "\[(\d+), \d*\.?\d+\]"))

    def __init__(self, pattern, type=float):
        self._regex = re.compile(pattern)
        self._samples = []
        self._type = type

    def parse(self, line):
        capture = self._regex.search(line)
        if capture is not None and capture.group(1) is not None:
            self._samples.append(self._type(capture.group(1)))

    def intervals(self):
        if len(self._samples) < 2:
            raise Exception("Not enough samples to calculate intervals!")
        intervals = []
        previous = self._samples[0]
        i = 1
        while i < len(self._samples):
            intervals.append(self._samples[i] - previous)
            previous = self._samples[i]
            i += 1
        return intervals

    def samples(self):
        return self._samples

def dropped_packet_summary(data, aliases=None):
    prefix = "\[INFO\] \[\d*\.?\d+\] \[.*\]: "
    alphanumeric = "[a-zA-Z0-9_]*"
    sending_type_parser = Parser(f"{prefix}Sending ({alphanumeric})", type=str)
    data.read(sending_type_parser.parse)
    sent_counts = {}
    for message_type in sending_type_parser.samples():
        if aliases:
            message_type = aliases.get(message_type, message_type)
        if not message_type.startswith("_"):
            sent_counts[message_type] = sent_counts.get(message_type, 0) + 1
    receiving_type_parser = Parser(f"{prefix}Received ({alphanumeric})", type=str)
    data.read(receiving_type_parser.parse)
    received_counts = {}
    for message_type in receiving_type_parser.samples():
        if aliases:
            message_type = aliases.get(message_type, message_type)
        if not message_type.startswith("_"):
            received_counts[message_type] = received_counts.get(message_type, 0) + 1
    for message_type, count in sent_counts.items():
        dropped = count - received_counts.get(message_type, 0)
        print(f"{message_type}: {dropped * 100 / count}%")
    return sent_counts, received_counts

def dropped_packet_percentage(data):
    sent_counts, received_counts = data
    sent     = sum(x for _, x in sent_counts.items())
    received = sum(x for _, x in received_counts.items())
    return received * 100 / sent
