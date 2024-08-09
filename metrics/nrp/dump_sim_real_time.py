#!/usr/bin/env python3
# Copyright (c) 2024 Lee Perry

import logging
from subprocess import Popen, PIPE, STDOUT

def main():

    logging.basicConfig(format="[%(levelname)s] [%(created)f] [%(name)s]: %(message)s",
                        level=logging.INFO)
    logger = logging.getLogger("nrp-sim-real-time-logger")

    active      = False
    seconds     = 0
    nanoseconds = 0

    p = Popen('gz topic -e /gazebo/default/world_stats',
              stdout=PIPE,
              stderr=STDOUT,
              shell=True)

    while True:
        line = p.stdout.readline()
        if not line: break
        line = str(line, "utf-8").strip()
        if line.startswith("sim_time"):
            active = True
        elif line.startswith("sec"):
            if active:
                seconds = int(line.split(":")[1].strip())
        elif line.startswith("nsec"):
            if active:
                nanoseconds = int(line.split(":")[1].strip())
        if line.startswith("}"):
            if active:
                sim_time = seconds + (nanoseconds / 1_000_000_000)
                logger.info(f"Simulated time: {sim_time}")
                active = False

    logger.info("Process finished")

if __name__ == '__main__':
    main()
