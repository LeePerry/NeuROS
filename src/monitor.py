#!/usr/bin/env python3
# Copyright (c) 2023 Lee Perry

"""
This module is responsible for calculating system load and throughput
performance metrics, and logging them to a file.
"""

import logging
import psutil
import time

# https://thepythoncode.com/article/make-a-network-usage-monitor-in-python

def monitor_system_load():
    logging.basicConfig(format="[%(levelname)s] [%(created)f] [%(name)s]: %(message)s", 
                        level=logging.INFO)
    logger = logging.getLogger("system-load-monitor")
    bytes_recv = 0
    bytes_sent = 0

    while True:
        time.sleep(1)

        # CPU - Percentage utilisation per core
        logger.info(f"CPU: {psutil.cpu_percent(percpu=True)}")

        # Network - Upload / Download speeds
        network_stats = psutil.net_io_counters()
        recv_speed = network_stats.bytes_recv - bytes_recv
        send_speed = network_stats.bytes_sent - bytes_sent
        bytes_recv = network_stats.bytes_recv
        bytes_sent = network_stats.bytes_sent
        logger.info(f"Network: {recv_speed}, {send_speed}")

if __name__ == '__main__':
    monitor_system_load()
