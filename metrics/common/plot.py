# Copyright (c) 2024 Lee Perry

import matplotlib.pyplot as plt
import numpy as np

main_colour = "green"

def histogram(data, x_axis, relative_frequency=False, bins=20):
    fig = plt.figure()
    ax = fig.add_subplot(111)
    if relative_frequency:
        data = np.array(data)
        ax.hist(data, bins=bins, 
                weights=np.zeros_like(data) + 100.0 / data.size, color=main_colour)
        ax.set_ylabel('Frequency (%)', size=12)
    else:
        ax.hist(data, bins=bins, color=main_colour)
        ax.set_ylabel('Count', size=12)
    ax.set_xlabel(x_axis, size=12)
    fig.tight_layout()
    plt.show()

def line(x, y, x_axis, y_axis):
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.plot(x, y)
    ax.set_xlabel(x_axis, size=12)
    ax.set_ylabel(y_axis, size=12)
    fig.tight_layout()
    plt.show()
