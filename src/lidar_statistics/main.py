#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
import math


if __name__ == "__main__":
    # Load CSV data
    raw_data_table = np.loadtxt("lidar_data.csv", delimiter=",", skiprows=1)

    # Extract sensor value
    raw_data_array = raw_data_table.T[1]

    # Formatting data(remove data less than 6500
    # and convert unit from mili metre to metre)
    data_array = raw_data_array[raw_data_array > 6500] / 1000

    mean = round(data_array.mean(), 3)
    std = round(data_array.std(), 3)

    data_bins = 150

    data_range = round(max(data_array) - min(data_array), 3)

    fig = plt.figure()
    ax = fig.add_subplot(
            111,
            xlabel="distance",
            ylabel="frequency",
            xlim=(6.75, 7.0)
            )

    ax.hist(data_array, bins=data_bins, density=True, label="Measured Data")

    ax.text(0.025, 0.95, "bins = " + str(data_bins), transform=ax.transAxes)
    ax.text(0.025, 0.9, r"$\mu$ = " + str(mean), transform=ax.transAxes)
    ax.text(0.025, 0.85, r"$\sigma$ = " + str(std), transform=ax.transAxes)
    ax.text(0.025, 0.8, "$R$ = " + str(data_range), transform=ax.transAxes)

    xs = np.arange(6.75, 7.0, 0.001)
    ys = np.exp(-(xs - mean)**2 / (2 * std**2)) / np.sqrt(2 * math.pi * std**2)

    ax.plot(xs, ys, label="Gaussian approximation")

    ax.legend(loc=1)

    # plt.show()
    plt.savefig("lidar_data.pdf")
