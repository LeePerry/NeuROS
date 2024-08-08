#!/usr/bin/env python3
# Copyright (c) 2024 Lee Perry

"""
This module replots all graphs and recalculates all metrics from the existing data.
"""

import sys

from metric_1_tick import process_data as plot1
from metric_2_synchronisation import process_data as plot2
from metric_3_physics_simulation import process_data as plot3
from metric_4_neurorobotics import process_data as plot4

if __name__ == '__main__':
    with open("results_data/console_output.txt", 'w') as sys.stdout:
        print("\n\n #### TICK CONTROLLER ####")
        plot1()
        print("\n\n #### SYNCHRONISATION ####")
        plot2()
        print("\n\n #### PHYSICS SIMULATION ####")
        plot3()
        print("\n\n #### COMPLETE NEUROROBOTICS EXPERIMENT ####")
        plot4()
