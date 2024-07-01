#!/usr/bin/env python3

from reader import Reader
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import curve_fit


def calculate_sos(reader: Reader):
    distance_tof = reader.get_data('/bluerov00/distance_tof')
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

    # linear
    # # define the true objective function
    # def objective(x, a, b):
    #     return a * x + b

    # # curve fit
    # popt, _ = curve_fit(objective, distances, tof)
    # # summarize the parameter values
    # a, b = popt
    # print('y = %.6f * x + %.6f' % (a, b))
    # x_regr = np.arange(0, 15, 1)
    # y_regr = objective(x_regr, a, b)

    # proportional
    # define the true objective function
    def objective(x, a):
        return a * x

    # curve fit
    popt, _ = curve_fit(objective, distances, tof)
    # summarize the parameter values
    a = popt
    print('y = %.6f * x' % (a))
    x_regr = np.arange(0, 15, 1)
    y_regr = objective(x_regr, a)

    SOS = 1 / a
    print('SOS = %.2f' % (SOS))

    plt.figure()
    plt.scatter(distances, tof)
    plt.plot(x_regr, y_regr, '--', color='red')
    plt.grid(True)
    plt.legend()
    plt.show()


def main():
    reader = Reader('init_sos_test')
    calculate_sos(reader)


if __name__ == '__main__':
    main()
