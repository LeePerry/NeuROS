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
    
    while True:
        time.sleep(1)
        logger.info(f"CPU: {psutil.cpu_percent(percpu=True)}")

if __name__ == '__main__':
    monitor_system_load()
