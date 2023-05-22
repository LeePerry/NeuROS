# Copyright (c) 2023 Lee Perry

"""
This module is responsible for calculating system load and throughput
performance metrics, and logging them to a file.
"""

import psutil
import time

def monitor_system_load(config, filename=None, interval_ms=1000):

    if filename is None:
        gmt = time.gmtime()
        mid = time.strftime("%Y_%m_%d_%H_%M_%S", gmt)
        filename = f"{config.project_dir}/neuros_monitoring_{mid}.csv"

    with open(filename, "x") as f:
        while True:
            f.write("blah\n") # psutil metric
            time.sleep(interval_ms)
