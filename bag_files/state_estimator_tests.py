#!/usr/bin/env python3

from reader import Reader
import matplotlib.pyplot as plt
import numpy as np


def plot_ground_truth_vs_estimate(reader: Reader):
    ground_truth_data = reader.get_data('/bluerov01/vehicle')
    n_messages = len(ground_truth_data)
    x_gt = np.zeros([n_messages])
    y_gt = np.zeros([n_messages])
    t_gt = np.zeros([n_messages])

    i = 0
    for msg, time_received in ground_truth_data:
        x_gt[i] = msg.north
        y_gt[i] = msg.east
        t_gt[i] = time_received * 1e-9
        i += 1

    state_estimate_data = reader.get_data('/bluerov01/state_estimate')
    n_messages = len(state_estimate_data)
    x_se = np.zeros([n_messages])
    y_se = np.zeros([n_messages])
    t_se = np.zeros([n_messages])

    i = 0
    for msg, time_received in state_estimate_data:
        x_se[i] = msg.pose.pose.position.x
        y_se[i] = msg.pose.pose.position.y
        t_se[i] = time_received * 1e-9
        i += 1

    # ground truth values interpolated linearily and mapped to timestamps of state estimate
    x_gt_norm = np.interp(t_se, t_gt, x_gt)
    x_error = abs(x_se - x_gt_norm)

    y_gt_norm = np.interp(t_se, t_gt, y_gt)
    y_error = abs(y_se - y_gt_norm)

    position_error = np.sqrt(
        np.power(x_se - x_gt_norm, 2) + np.power(y_se - y_gt_norm, 2))

    # CDF
    # sort the data:
    position_error_sorted = np.sort(position_error)
    # calculate the proportional values of samples
    p = 1. * np.arange(len(position_error)) / (len(position_error) - 1)

    plt.figure()
    # Track
    plt.plot(x_gt, y_gt, label='Ground Truth RTK')
    plt.plot(x_se, y_se, label='State Estimate')

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

    plt.grid(True)
    plt.legend()
    plt.show()


def main():
    reader = Reader('ekf_convergence')
    plot_ground_truth_vs_estimate(reader)


if __name__ == '__main__':
    main()
