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

    # state_estimate_data = reader.get_data('/bluerov01/state_estimate')
    # n_messages = len(state_estimate_data)
    # x_se = np.zeros([n_messages])
    # y_se = np.zeros([n_messages])
    # t_se = np.zeros([n_messages])

    # i = 0
    # for msg, time_received in state_estimate_data:
    #     x_se[i] = msg.pose.pose.position.x
    #     y_se[i] = msg.pose.pose.position.y
    #     t_se[i] = time_received * 1e-9
    #     i += 1

    # # measured distances
    # acoustic_distance_data = reader.get_data('/bluerov01/acoustic_distance')
    # n_messages = len(acoustic_distance_data)
    # time = np.zeros([n_messages])
    # id = np.zeros([n_messages])
    # distance = np.zeros([n_messages])

    # i = 0
    # for msg, time_received in acoustic_distance_data:
    #     id[i] = msg.id
    #     distance[i] = msg.range
    #     time[i] = time_received * 1e-9
    #     i += 1

    # filter_arr_0 = filter_id(0, id)
    # distance_0 = distance[filter_arr_0]
    # time_0 = time[filter_arr_0]
    # filter_arr_1 = filter_id(1, id)
    # distance_1 = distance[filter_arr_1]
    # time_1 = time[filter_arr_1]
    # filter_arr_2 = filter_id(2, id)
    # distance_2 = distance[filter_arr_2]
    # time_2 = time[filter_arr_2]

    # # ground truth values interpolated linearily and mapped to timestamps of state estimate
    # x_gt_norm = np.interp(t_se, t_gt, x_gt)
    # x_error = abs(x_se - x_gt_norm)

    # y_gt_norm = np.interp(t_se, t_gt, y_gt)
    # y_error = abs(y_se - y_gt_norm)

    # position_error = np.sqrt(
    #     np.power(x_se - x_gt_norm, 2) + np.power(y_se - y_gt_norm, 2))

    # # CDF
    # # sort the data:
    # position_error_sorted = np.sort(position_error)
    # # calculate the proportional values of samples
    # p = 1. * np.arange(len(position_error)) / (len(position_error) - 1)

    ######################################### plots ###########################################
    plt.figure(1)

    # Track
    plt.plot(x_gt, y_gt, label='Ground Truth RTK')
    # plt.plot(x_se, y_se, label='State Estimate')
    plt.plot(buoy_2_x, buoy_2_y, label='buoy_2')
    plt.plot(buoy_3_x, buoy_3_y, label='buoy_3')
    plt.plot(buoy_5_x, buoy_5_y, label='buoy_5')
    plt.xlabel('east')
    plt.ylabel('north')
    plt.title('RTK Tracks')

    # x coordinate gt vs se
    # plt.plot(t_se,x_se)
    # plt.plot(t_gt,x_gt)

    # x coordinate gt vs se at same timestamps
    # plt.plot(t_se, x_se)
    # plt.plot(t_se, x_gt_norm)

    # x error
    # plt.plot(t_se, x_error)
    # plt.plot(t_se, position_error)

    # CDF position error
    # plt.plot(p, position_error_sorted)

    # if state_estimate is bad, check if at least distances were logical
    # plt.scatter(time, distance)
    # plt.scatter(distance, id)
    # plt.scatter(time_0, distance_0)

    plt.grid(True)
    plt.legend()
    plt.show()

    figure, axis = plt.subplots(2, 1)
    axis[0].plot(t_gt_datetime, buoy_2_x)
    axis[0].set_title("x value")
    axis[0].set_xlabel('time')
    axis[0].set_ylabel('x')
    axis[0].grid()

    axis[1].plot(t_gt_datetime, buoy_2_y)
    axis[1].set_title("y value")
    axis[1].set_xlabel('time')
    axis[1].set_ylabel('y value')
    axis[1].grid()
    plt.show()


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
