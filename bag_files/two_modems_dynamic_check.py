#!/usr/bin/env python3

from reader import Reader
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import curve_fit


def calculate_sos(reader: Reader):
    distance_tof = reader.get_data('/bluerov01/distance_tof')
    n_messages = len(distance_tof)
    timestamps = np.zeros([n_messages])
    distances = np.zeros([n_messages])
    tof = np.zeros([n_messages])

    i = 0
    for msg, time_received in distance_tof:
        timestamps[i] = time_received * 1e-9
        distances[i] = msg.distance
        tof[i] = msg.tof
        i += 1

    plt.figure()
    plt.scatter(timestamps, tof)
    plt.grid(True)
    plt.xlabel('time')
    plt.ylabel('tof')
    plt.show()


def main():
    reader = Reader('two_modems_dynamic_v4')
    calculate_sos(reader)


if __name__ == '__main__':
    main()
