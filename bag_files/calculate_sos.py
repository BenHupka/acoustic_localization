#!/usr/bin/env python3

from reader import Reader
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import curve_fit
from datetime import datetime, timezone
import math


def calculate_sos(reader: Reader):
    t_start = 0.0
    t_end = 75.0

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

    t_0 = t_gt[0]
    # timestamps in Sekunden und Start auf Null
    t_gt_norm = (t_gt - t_0)
    timestamps_norm = (timestamps - t_0)

    timestamps, tof, timestamps_norm = crop_data2(timestamps, tof,
                                                  timestamps_norm, t_end)

    x_gt, y_gt, t_gt, t_gt_norm = crop_data3(x_gt, y_gt, t_gt, t_gt_norm, t_end)

    distances_ctd_sos = 1483 * tof

    # ################################## !!!!!!!!!!!!!! Ideen für Korrektur RTK
    # x_gt -= 1.5
    # y_gt -= 1.5

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
    distance_pressure_sensor_hydrophone = 0.52  # bei two_modems_dynamic_v4 und reihum 0.52, sonst -0.15
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
    error = distances_ctd_sos - distances_rtk
    error_abs = abs(error)

    #RMS error
    n = len(error)
    E = 0.0
    for i in range(n):
        E += error[i]**2
    rms = math.sqrt(E / n)
    print(f'RMS error = {rms}')

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

    ####################################################### export ###########
    # compress
    t_gt_norm_compressed = np.linspace(0, 75, 75)
    distance_gt_1_compressed = np.interp(t_gt_norm_compressed, t_gt_norm,
                                         distance_gt_1)

    # data = np.hstack([
    #     timestamps_norm.reshape(-1, 1),
    #     distances_ctd_sos.reshape(-1, 1),
    #     error.reshape(-1, 1)
    # ])
    # np.savetxt('export/dynamic_v4_acoustic_distances.csv',
    #            data,
    #            delimiter=',',
    #            header='timestamps_norm, distances_ctd, error',
    #            comments='')

    # data = np.hstack([
    #     t_gt_norm_compressed.reshape(-1, 1),
    #     distance_gt_1_compressed.reshape(-1, 1)
    # ])
    # np.savetxt('export/dynamic_v4_rtk_distances.csv',
    #            data,
    #            delimiter=',',
    #            header='t_gt_norm, distance_gt_1',
    #            comments='')
    ##########################################################################

    figure, axis = plt.subplots(3, 1)
    axis[2].scatter(distances_rtk, tof)  #distances_rtk, tof
    axis[2].plot(x_regr, y_regr, '--',
                 color='red')  #x_regr, y_regr, '--', color='red'
    axis[2].set_title("Best fit through tof over distance")
    axis[2].set_xlabel('distance')
    axis[2].set_ylabel('tof')
    axis[2].grid()

    axis[0].scatter(timestamps_norm,
                    distances_ctd_sos,
                    label='acoustic distances')
    # axis[1].plot(timestamps, distances_rtk, label='rtk distances ac')
    axis[0].plot(t_gt_norm_compressed,
                 distance_gt_1_compressed,
                 label='rtk distances',
                 color='green')
    axis[0].set_title("Acoustic vs rtk distances")
    axis[0].set_xlabel('timestamp')
    axis[0].set_ylabel('distance')
    axis[0].set_xlim([t_gt_norm[0], t_gt_norm[-1]])
    axis[0].grid()

    axis[1].scatter(
        timestamps_norm, error, label='error',
        color='red')  #timestamps_norm, error, label='error', color='red'
    axis[1].set_title("ranging error")
    axis[1].set_xlabel('timestamp')
    axis[1].set_ylabel('error')
    axis[1].set_xlim([t_gt_norm[0], t_gt_norm[-1]])
    axis[1].grid()
    plt.show()

    # plt.figure()
    # # plt.plot(t_gt_norm, ROV_hydrophone_depth_norm)
    # plt.plot(t_pressure, pressure)
    # plt.show()


def crop_data(data, time, t1):
    filter_arr = []
    for element in time:
        if element <= t1:
            filter_arr.append(True)
        else:
            filter_arr.append(False)
    data_filt = data[filter_arr]
    time_filt = time[filter_arr]
    return data_filt, time_filt


def crop_data2(data1, data2, time, t1):
    filter_arr = []
    for element in time:
        if element < t1:
            filter_arr.append(True)
        else:
            filter_arr.append(False)
    data1_filt = data1[filter_arr]
    data2_filt = data2[filter_arr]
    time_filt = time[filter_arr]
    return data1_filt, data2_filt, time_filt


def crop_data3(data1, data2, data3, time, t1):
    filter_arr = []
    for element in time:
        if element < t1:
            filter_arr.append(True)
        else:
            filter_arr.append(False)
    data1_filt = data1[filter_arr]
    data2_filt = data2[filter_arr]
    data3_filt = data3[filter_arr]
    time_filt = time[filter_arr]
    return data1_filt, data2_filt, data3_filt, time_filt


def main():
    reader = Reader('two_modems_dynamic_v4'
                    )  # ab two_modems_dynamic_v4 andere hydrophone depth!!!
    calculate_sos(reader)


if __name__ == '__main__':
    main()
