#!/usr/bin/env python3

from reader import Reader
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import curve_fit
from datetime import datetime, timezone


def calculate_sos(reader: Reader):
    distance_tof = reader.get_data('/bluerov01/distance_tof')
    n_messages = len(distance_tof)
    timestamps = np.zeros([n_messages])
    timestamps_datetime = np.array([])
    distances_rtk_corrupt = np.zeros(
        [n_messages])  # corrupt because pressure wasn't subscribed properly
    tof = np.zeros([n_messages])

    i = 0
    for msg, time_received in distance_tof:
        timestamps[i] = time_received * 1e-9
        timestamps_datetime = np.append(
            timestamps_datetime,
            datetime.fromtimestamp(timestamps[i], timezone.utc))
        distances_rtk_corrupt[i] = msg.distance
        tof[i] = msg.tof
        i += 1

    distances_ctd_sos = 1483 * tof

    ground_truth_data = reader.get_data('/bluerov01/vehicle')
    n_messages = len(ground_truth_data)
    x_gt = np.zeros([n_messages])
    y_gt = np.zeros([n_messages])
    t_gt = np.zeros([n_messages])
    t_gt_datetime = np.array([])

    i = 0
    for msg, time_received in ground_truth_data:
        x_gt[i] = msg.east
        y_gt[i] = msg.north
        t_gt[i] = time_received * 1e-9
        t_gt_datetime = np.append(t_gt_datetime,
                                  datetime.fromtimestamp(t_gt[i], timezone.utc))
        i += 1

    pressure_data = reader.get_data('/bluerov01/pressure')
    n_messages = len(pressure_data)
    pressure = np.zeros([n_messages])
    t_pressure = np.zeros([n_messages])

    i = 0
    for msg, time_received in pressure_data:
        pressure[i] = msg.fluid_pressure
        t_pressure[i] = time_received * 1e-9
        i += 1

    buoy_2_rtk = reader.get_data('/bluerov01/buoy_2')
    n_messages = len(buoy_2_rtk)
    buoy_2_x = np.zeros([n_messages])
    buoy_2_y = np.zeros([n_messages])
    buoy_2_t = np.zeros([n_messages])

    i = 0
    for msg, time_received in buoy_2_rtk:
        buoy_2_x[i] = msg.east
        buoy_2_y[i] = msg.north
        buoy_2_t[i] = time_received * 1e-9
        i += 1

    # calculated hydrophone depth
    pressure_surface = 100000.0
    density_water = 1e4
    distance_pressure_sensor_hydrophone = -0.25  # bei two_modems_dynamic_v4 und reihum 0.42, sonst -0.25
    hydrophone_depth = np.zeros(len(pressure))
    for i, p in enumerate(pressure):
        hydrophone_depth[i] = (
            pressure[i] - pressure_surface
        ) / density_water + distance_pressure_sensor_hydrophone

    # buoy rtk values and depths interpolated linearily and mapped to timestamps of vehicle rtk
    buoy_2_x_norm = np.interp(t_gt, buoy_2_t, buoy_2_x)
    buoy_2_y_norm = np.interp(t_gt, buoy_2_t, buoy_2_y)
    ROV_hydrophone_depth_norm = np.interp(t_gt, t_pressure, hydrophone_depth)
    buoy_hydrophone_depth_norm = 1.55 * np.ones(len(ROV_hydrophone_depth_norm))

    # rtk distances
    distance_gt_1 = np.sqrt(
        np.power(buoy_2_x_norm - x_gt, 2) + np.power(buoy_2_y_norm - y_gt, 2) +
        np.power(buoy_hydrophone_depth_norm - ROV_hydrophone_depth_norm, 2))

    # rtk distances mapped to timestamps of tof measurements
    distances_rtk = np.interp(timestamps, t_gt, distance_gt_1)

    # rtk and ac ranging error
    error = abs(distances_rtk - distances_ctd_sos)

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
    popt, _ = curve_fit(objective, distances_rtk, tof)
    # summarize the parameter values
    a = popt
    print('y = %.6f * x' % (a))
    x_regr = np.arange(0, 15, 1)
    y_regr = objective(x_regr, a)

    SOS = 1 / a
    print('SOS = %.2f' % (SOS))

    figure, axis = plt.subplots(3, 1)
    axis[0].scatter(distances_rtk, tof)
    axis[0].plot(x_regr, y_regr, '--', color='red')
    axis[0].set_title("Best fit through tof over distance")
    axis[0].set_xlabel('distance')
    axis[0].set_ylabel('tof')
    axis[0].grid()

    axis[1].scatter(timestamps_datetime,
                    distances_ctd_sos,
                    label='acoustic distances')
    # axis[1].plot(timestamps, distances_rtk, label='rtk distances ac')
    axis[1].plot(t_gt_datetime,
                 distance_gt_1,
                 label='rtk distances',
                 color='green')
    axis[1].set_title("Acoustic vs rtk distances")
    axis[1].set_xlabel('timestamp')
    axis[1].set_ylabel('distance')
    axis[1].set_xlim([t_gt_datetime[0], t_gt_datetime[-1]])
    axis[1].grid()

    axis[2].scatter(timestamps_datetime, error, label='error', color='red')
    axis[2].set_title("ranging error")
    axis[2].set_xlabel('timestamp')
    axis[2].set_ylabel('error')
    axis[2].set_xlim([t_gt_datetime[0], t_gt_datetime[-1]])
    axis[2].grid()
    plt.show()


def main():
    reader = Reader(
        'init_sos')  # ab two_modems_dynamic_v4 andere hydrophone depth!!!
    calculate_sos(reader)


if __name__ == '__main__':
    main()
