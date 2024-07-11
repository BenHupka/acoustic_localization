#!/usr/bin/env python3

from reader import Reader
import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime, timezone


def plot_ground_truth_vs_estimate(reader: Reader):
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

    buoy_3_rtk = reader.get_data('/bluerov01/buoy_3')
    n_messages = len(buoy_3_rtk)
    buoy_3_x = np.zeros([n_messages])
    buoy_3_y = np.zeros([n_messages])
    buoy_3_t = np.zeros([n_messages])

    i = 0
    for msg, time_received in buoy_3_rtk:
        buoy_3_x[i] = msg.east
        buoy_3_y[i] = msg.north
        buoy_3_t[i] = time_received * 1e-9
        i += 1

    buoy_5_rtk = reader.get_data('/bluerov01/buoy_5')
    n_messages = len(buoy_5_rtk)
    buoy_5_x = np.zeros([n_messages])
    buoy_5_y = np.zeros([n_messages])
    buoy_5_t = np.zeros([n_messages])

    i = 0
    for msg, time_received in buoy_5_rtk:
        buoy_5_x[i] = msg.east
        buoy_5_y[i] = msg.north
        buoy_5_t[i] = time_received * 1e-9
        i += 1

    # measured distances
    acoustic_distance_data = reader.get_data('/bluerov01/acoustic_distance')
    n_messages = len(acoustic_distance_data)
    time = np.zeros([n_messages])
    id = np.zeros([n_messages])
    distance = np.zeros([n_messages])
    time_datetime = np.array([])

    i = 0
    for msg, time_received in acoustic_distance_data:
        id[i] = msg.id
        distance[
            i] = msg.range * 1483 / 1500  # range war mit SOS 1500 berechnet
        time[i] = time_received * 1e-9
        time_datetime = np.append(time_datetime,
                                  datetime.fromtimestamp(time[i], timezone.utc))
        i += 1

    filter_arr_0 = filter_id(0, id)
    distance_ac_0 = distance[filter_arr_0]
    time_ac_0 = time[filter_arr_0]
    time_datetime_ac_0 = time_datetime[filter_arr_0]
    filter_arr_1 = filter_id(1, id)
    distance_ac_1 = distance[filter_arr_1]
    time_ac_1 = time[filter_arr_1]
    time_datetime_ac_1 = time_datetime[filter_arr_1]
    filter_arr_2 = filter_id(2, id)
    distance_ac_2 = distance[filter_arr_2]
    time_ac_2 = time[filter_arr_2]
    time_datetime_ac_2 = time_datetime[filter_arr_2]

    # calculated hydrophone depth
    pressure_surface = 100000.0
    density_water = 1e4
    distance_pressure_sensor_hydrophone = 0.42
    hydrophone_depth = np.zeros(len(pressure))
    for i, p in enumerate(pressure):
        hydrophone_depth[i] = (
            pressure[i] - pressure_surface
        ) / density_water + distance_pressure_sensor_hydrophone

    # buoy rtk values and depths interpolated linearily and mapped to timestamps of vehicle rtk
    buoy_5_x_norm = np.interp(t_gt, buoy_5_t, buoy_5_x)
    buoy_5_y_norm = np.interp(t_gt, buoy_5_t, buoy_5_y)
    buoy_2_x_norm = np.interp(t_gt, buoy_2_t, buoy_2_x)
    buoy_2_y_norm = np.interp(t_gt, buoy_2_t, buoy_2_y)
    buoy_3_x_norm = np.interp(t_gt, buoy_3_t, buoy_3_x)
    buoy_3_y_norm = np.interp(t_gt, buoy_3_t, buoy_3_y)
    ROV_hydrophone_depth_norm = np.interp(t_gt, t_pressure, hydrophone_depth)
    buoy_hydrophone_depth_norm = 1.5 * np.ones(len(ROV_hydrophone_depth_norm))

    # rtk distances
    distance_gt_0 = np.sqrt(
        np.power(buoy_5_x_norm - x_gt, 2) + np.power(buoy_5_y_norm - y_gt, 2) +
        np.power(buoy_hydrophone_depth_norm - ROV_hydrophone_depth_norm, 2))
    distance_gt_1 = np.sqrt(
        np.power(buoy_2_x_norm - x_gt, 2) + np.power(buoy_2_y_norm - y_gt, 2) +
        np.power(buoy_hydrophone_depth_norm - ROV_hydrophone_depth_norm, 2))
    distance_gt_2 = np.sqrt(
        np.power(buoy_3_x_norm - x_gt, 2) + np.power(buoy_3_y_norm - y_gt, 2) +
        np.power(buoy_hydrophone_depth_norm - ROV_hydrophone_depth_norm, 2))

    # rtk distances interpolated linearily and mapped to timestamps of acoustic measurements
    distance_gt_0_at_ranging = np.interp(time_ac_0, t_gt, distance_gt_0)
    distance_gt_1_at_ranging = np.interp(time_ac_1, t_gt, distance_gt_1)
    distance_gt_2_at_ranging = np.interp(time_ac_2, t_gt, distance_gt_2)

    # distance errors
    distance_0_error = abs(distance_gt_0_at_ranging - distance_ac_0)
    distance_1_error = abs(distance_gt_1_at_ranging - distance_ac_1)
    distance_2_error = abs(distance_gt_2_at_ranging - distance_ac_2)

    # position_error = np.sqrt(
    #     np.power(x_se - x_gt_norm, 2) + np.power(y_se - y_gt_norm, 2))

    # # CDF
    # # sort the data:
    # position_error_sorted = np.sort(position_error)
    # # calculate the proportional values of samples
    # p = 1. * np.arange(len(position_error)) / (len(position_error) - 1)

    ######################################### plots ###########################################
    # RTK und akustische Distanzen zu allen 3 Bojen
    figure, axis = plt.subplots(3, 1)
    axis[0].plot(t_gt_datetime, distance_gt_0, color='green')
    axis[0].scatter(time_datetime_ac_0, distance_ac_0)
    axis[0].set_title("Distance buoy_5")
    axis[0].set_xlabel('timestamp')
    axis[0].set_ylabel('distance')
    axis[0].grid()

    axis[1].plot(t_gt_datetime, distance_gt_1, color='green')
    axis[1].scatter(time_datetime_ac_1, distance_ac_1)
    axis[1].set_title("Distance buoy_2")
    axis[1].set_xlabel('timestamp')
    axis[1].set_ylabel('distance')
    axis[1].grid()

    axis[2].plot(t_gt_datetime, distance_gt_2, color='green')
    axis[2].scatter(time_datetime_ac_2, distance_ac_2)
    axis[2].set_title("Distance buoy_3")
    axis[2].set_xlabel('timestamp')
    axis[2].set_ylabel('distance')
    axis[2].grid()

    # axis[3].plot(t_gt_datetime, ROV_hydrophone_depth_norm)
    # axis[3].plot(t_pressure, pressure)
    # axis[3].set_title("Depth")
    # axis[3].grid()
    # axis[3].plot(buoy_5_x, buoy_5_y)
    # axis[3].set_title("Depth")
    # axis[3].grid()

    plt.show()

    # x und y von ROV und Boje 3
    # figure, axis = plt.subplots(4, 1)
    # axis[0].plot(t_gt_datetime, x_gt)
    # axis[0].set_title("ROV x")
    # axis[0].set_xlabel('timestamp')
    # axis[0].set_ylabel('x')
    # axis[0].grid()

    # axis[1].plot(t_gt_datetime, y_gt)
    # axis[1].set_title("ROV y")
    # axis[1].set_xlabel('timestamp')
    # axis[1].set_ylabel('y')
    # axis[1].grid()

    # axis[2].plot(t_gt_datetime, buoy_3_x_norm)
    # axis[2].set_title("buoy 3 x")
    # axis[2].set_xlabel('timestamp')
    # axis[2].set_ylabel('x')
    # axis[2].grid()

    # axis[3].plot(t_gt_datetime, buoy_3_y_norm)
    # axis[3].set_title("buoy 3 y")
    # axis[3].set_xlabel('timestamp')
    # axis[3].set_ylabel('y')
    # axis[3].grid()
    # plt.show()

    # single ranging points
    # plt.figure()
    # plt.scatter(time_datetime_ac_2[[0, 1]], distance_gt_2_at_ranging[[0, 1]])
    # plt.scatter(time_datetime_ac_2[[0, 1]], distance_ac_2[[0, 1]])
    # plt.grid()
    # plt.show()

    # ranging errors
    figure, axis = plt.subplots(3, 1)
    axis[0].scatter(time_datetime_ac_0, distance_0_error)
    axis[0].set_title("error buoy_5")
    axis[0].set_xlabel('timestamp')
    axis[0].set_ylabel('error')
    axis[0].grid()

    axis[1].scatter(time_datetime_ac_1, distance_1_error)
    axis[1].set_title("error buoy_2")
    axis[1].set_xlabel('timestamp')
    axis[1].set_ylabel('error')
    axis[1].grid()

    axis[2].scatter(time_datetime_ac_2, distance_2_error)
    axis[2].set_title("error buoy_3")
    axis[2].set_xlabel('timestamp')
    axis[2].set_ylabel('error')
    axis[2].grid()
    plt.show()

    # pressure and depth
    # figure, axis = plt.subplots(2, 1)
    # axis[0].plot(t_pressure, pressure)
    # axis[0].set_title("pressure")
    # axis[0].set_xlabel('timestamp')
    # axis[0].set_ylabel('pressure')
    # axis[0].grid()

    # axis[1].scatter(t_gt_datetime, ROV_hydrophone_depth_norm)
    # axis[1].set_title("error buoy_2")
    # axis[1].set_xlabel('timestamp')
    # axis[1].set_ylabel('depth')
    # axis[1].grid()
    # plt.show()


def filter_id(id, arr):
    filter_arr = []
    for element in arr:
        if element == id:
            filter_arr.append(True)
        else:
            filter_arr.append(False)
    return filter_arr


def main():
    reader = Reader(
        'reihum_modem_test_mit_state_estimator_vergleich_mit_chris_logs')
    plot_ground_truth_vs_estimate(reader)


if __name__ == '__main__':
    main()
