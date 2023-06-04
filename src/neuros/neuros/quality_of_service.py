# Copyright (c) 2023 Lee Perry

import rclpy.qos as qos
from rclpy.qos import QoSProfile

def standard_quality():
    return QoSProfile(
        reliability = qos.ReliabilityPolicy.BEST_EFFORT,
        durability  = qos.DurabilityPolicy.TRANSIENT_LOCAL,
        history     = qos.HistoryPolicy.KEEP_LAST,
        depth       = 1)