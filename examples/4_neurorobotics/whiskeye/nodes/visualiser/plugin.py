# Copyright (c) 2024 Lee Perry

import numpy as np
import time

from neuros.hooks import neuros_initialise, neuros_function
from neuros.visualise import VisualiseRotations

GROUND_TRUTH = "Ground Truth"
SNN_OUTPUT   = "Spiking Neural Network"

def ring_mean_activity(data, centre = True):
    # Data is expected to be a single ring's activity history of shape (timesteps, ring_size)
    # Centre == False: rays are from 0 -> 2pi, half-open interval. Centre == True: rays are adjusted to project from halfway along their arc
    data = np.array(data)
    if data.ndim < 2:
        data = data.reshape(1, -1)

    ring_size = data.shape[1]
    arc_per_ring_segment = (2 * np.pi) / ring_size
    rays = np.repeat(np.arange(0, 2 * np.pi, arc_per_ring_segment).reshape(1, -1), data.shape[0], axis = 0)
    if centre:
        rays = rays + arc_per_ring_segment / 2

    rays_for_each_spike = np.empty(shape = (data.shape[0]), dtype = 'object')
    mean_activity = np.empty(shape = (data.shape[0]), dtype = 'float')
    for i in range(rays_for_each_spike.shape[0]):
        if len(np.nonzero(data[i,:])) > 0:
            rays_for_each_spike[i] = np.repeat(rays[i,:][data[i,:] > 0], data[i, :][data[i,:] > 0].astype('int'))
            mean_activity[i] = np.arctan2(np.mean(np.sin(rays_for_each_spike[i])), np.mean(np.cos(rays_for_each_spike[i]))) % (2 * np.pi)
        else:
            mean_activity[i] = 0
        mean_activity[np.isnan(mean_activity)] = 0
    mean_activity_ring_index = mean_activity * (ring_size / (2 * np.pi))
    return mean_activity_ring_index

@neuros_initialise()
def initialise(node):
    visualiser = VisualiseRotations('Head Direction', duration=60*5)
    visualiser.add_quaternion(GROUND_TRUTH, "g")
    visualiser.add_degrees(SNN_OUTPUT, "r")
    node.set_user_data((time.time(), visualiser))

@neuros_function(inputs="ground_truth")
def receive_ground_truth(node, ground_truth):
    started, visualiser = node.get_user_data()
    angle = ground_truth.pose.orientation
    visualiser.plot(GROUND_TRUTH, angle, time.time() - started)

@neuros_function(inputs="odom_estimate")
def receive_odom_estimate(node, estimate):
    started, visualiser = node.get_user_data()
    angle = ((estimate.data % 120) * (360 // 120) - 180)
    visualiser.plot(SNN_OUTPUT, angle, time.time() - started)
