# Copyright (c) 2024 Lee Perry

import re
import numpy as np

def basic_stats(data):
    print(f"Count: {len(data)}")
    print(f"Mean: {np.mean(data)}")
    print(f"Standard Deviation: {np.std(data)}")

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
        self._file = open(path, 'r')

    def read(self, parser):
        for line in self._file.readlines():
            parser(line)

class Parser:

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
