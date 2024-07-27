#!/usr/bin/env python3

from reader import Reader
import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime, timezone


def plot_ground_truth_vs_estimate(reader: Reader):
    ground_truth_data = reader.get_data('/bluerov00/ground_truth')
    n_messages = len(ground_truth_data)
    x_gt = np.zeros([n_messages])
    y_gt = np.zeros([n_messages])
    z_gt = np.zeros([n_messages])
    t_gt = np.zeros([n_messages])

    i = 0
    for msg, time_received in ground_truth_data:
        x_gt[i] = msg.vector.x
        y_gt[i] = msg.vector.y
        z_gt[i] = msg.vector.z
        t_gt[i] = time_received * 1e-9
        i += 1

    state_estimate_data = reader.get_data('/bluerov00/state_estimate_pp')
    n_messages = len(state_estimate_data)
    x_se = np.zeros([n_messages])
    y_se = np.zeros([n_messages])
    z_se = np.zeros([n_messages])
    t_se = np.zeros([n_messages])

    i = 0
    for msg, time_received in state_estimate_data:
        x_se[i] = msg.pose.pose.position.x
        y_se[i] = msg.pose.pose.position.y
        z_se[i] = msg.pose.pose.position.z
        t_se[i] = time_received * 1e-9
        i += 1

    # ground truth values interpolated linearily and mapped to timestamps of state estimate
    x_gt_norm = np.interp(t_se, t_gt, x_gt)
    x_error = x_se - x_gt_norm

    y_gt_norm = np.interp(t_se, t_gt, y_gt)
    y_error = y_se - y_gt_norm

    z_gt_norm = np.interp(t_se, t_gt, z_gt)
    z_error = z_se - z_gt_norm

    position_error = np.sqrt(
        np.power(x_se - x_gt_norm, 2) + np.power(y_se - y_gt_norm, 2) +
        np.power(z_se - z_gt_norm, 2))

    # CDF
    # sort the data:
    position_error_sorted = np.sort(position_error)
    # calculate the proportional values of samples
    p = 1. * np.arange(len(position_error)) / (len(position_error) - 1)

    ######################################### plots ###########################################
    figure, axis = plt.subplots(2, 1)
    axis[0].plot(x_gt, y_gt, label='Ground Truth')
    axis[0].scatter(x_se, y_se, label='State Estimate')
    axis[0].set_title("Track")
    axis[0].set_xlabel('x')
    axis[0].set_ylabel('y')
    axis[0].grid()
    axis[0].legend()

    axis[1].plot(p, position_error_sorted)
    axis[1].set_title("Localization error CDF")
    axis[1].set_xlabel('percentage')
    axis[1].set_ylabel('error')
    axis[1].grid()
    plt.show()

    # plt.plot()
    # # Track
    # plt.plot(x_gt, y_gt, label='Ground Truth')
    # plt.plot(x_se, y_se, label='State Estimate')
    # plt.xlabel('east')
    # plt.ylabel('north')
    # plt.title('RTK Tracks')

    # x coordinate gt vs se
    # plt.plot(t_se,x_se)
    # plt.plot(t_gt,x_gt)

    # x coordinate gt vs se at same timestamps
    # plt.plot(t_se, x_se)
    # plt.plot(t_se, x_gt_norm)

    # x error
    # plt.plot(t_se, z_error)
    # plt.plot(t_se, position_error)

    # CDF position error
    # plt.plot(p, position_error_sorted)

    # if state_estimate is bad, check if at least distances were logical
    # plt.scatter(time, distance)
    # plt.scatter(distance, id)
    # plt.scatter(time_0, distance_0)

    # plt.show()


def main():
    reader = Reader('ekf_test_nonoise')
    plot_ground_truth_vs_estimate(reader)


if __name__ == '__main__':
    main()
