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
    #data = common.data.Reader(data_path)
    pass

if __name__ == '__main__':
    create_data()
    process_data()
