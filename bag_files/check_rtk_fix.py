#!/usr/bin/env python3

from reader import Reader
import matplotlib.pyplot as plt
import numpy as np


def check_rtk_fix(reader: Reader):
    ground_truth_data = reader.get_data('/bluerov01/vehicle')
    n_messages = len(ground_truth_data)
    x_gt = np.zeros([n_messages])
    y_gt = np.zeros([n_messages])
    t_gt = np.zeros([n_messages])
    gt_gnss_fix_ok = np.zeros([n_messages])
    gt_carr_soln = np.zeros([n_messages])
    gt_rel_pos_valid = np.zeros([n_messages])

    i = 0
    for msg, time_received in ground_truth_data:
        x_gt[i] = msg.east
        y_gt[i] = msg.north
        t_gt[i] = time_received * 1e-9
        gt_gnss_fix_ok[i] = msg.gnss_fix_ok
        gt_carr_soln[i] = msg.carr_soln
        gt_rel_pos_valid[i] = msg.rel_pos_valid
        i += 1

    buoy_2_rtk = reader.get_data('/bluerov01/buoy_2')
    n_messages = len(buoy_2_rtk)
    buoy_2_x = np.zeros([n_messages])
    buoy_2_y = np.zeros([n_messages])
    buoy_2_t = np.zeros([n_messages])
    buoy_2_gnss_fix_ok = np.zeros([n_messages])
    buoy_2_carr_soln = np.zeros([n_messages])
    buoy_2_rel_pos_valid = np.zeros([n_messages])

    i = 0
    for msg, time_received in buoy_2_rtk:
        buoy_2_x[i] = msg.east
        buoy_2_y[i] = msg.north
        buoy_2_t[i] = time_received * 1e-9
        buoy_2_gnss_fix_ok[i] = msg.gnss_fix_ok
        buoy_2_carr_soln[i] = msg.carr_soln
        buoy_2_rel_pos_valid[i] = msg.rel_pos_valid
        i += 1

    buoy_3_rtk = reader.get_data('/bluerov01/buoy_3')
    n_messages = len(buoy_3_rtk)
    buoy_3_x = np.zeros([n_messages])
    buoy_3_y = np.zeros([n_messages])
    buoy_3_t = np.zeros([n_messages])
    buoy_3_gnss_fix_ok = np.zeros([n_messages])
    buoy_3_carr_soln = np.zeros([n_messages])
    buoy_3_rel_pos_valid = np.zeros([n_messages])

    i = 0
    for msg, time_received in buoy_3_rtk:
        buoy_3_x[i] = msg.east
        buoy_3_y[i] = msg.north
        buoy_3_t[i] = time_received * 1e-9
        buoy_3_gnss_fix_ok[i] = msg.gnss_fix_ok
        buoy_3_carr_soln[i] = msg.carr_soln
        buoy_3_rel_pos_valid[i] = msg.rel_pos_valid
        i += 1

    buoy_5_rtk = reader.get_data('/bluerov01/buoy_5')
    n_messages = len(buoy_5_rtk)
    buoy_5_x = np.zeros([n_messages])
    buoy_5_y = np.zeros([n_messages])
    buoy_5_t = np.zeros([n_messages])
    buoy_5_gnss_fix_ok = np.zeros([n_messages])
    buoy_5_carr_soln = np.zeros([n_messages])
    buoy_5_rel_pos_valid = np.zeros([n_messages])

    i = 0
    for msg, time_received in buoy_5_rtk:
        buoy_5_x[i] = msg.east
        buoy_5_y[i] = msg.north
        buoy_5_t[i] = time_received * 1e-9
        buoy_5_gnss_fix_ok[i] = msg.gnss_fix_ok
        buoy_5_carr_soln[i] = msg.carr_soln
        buoy_5_rel_pos_valid[i] = msg.rel_pos_valid
        i += 1

    figure, axis = plt.subplots(4, 1)
    axis[0].plot(buoy_5_t, buoy_5_gnss_fix_ok, color='orange')
    axis[0].plot(buoy_5_t, buoy_5_carr_soln, '*', color='red')
    axis[0].plot(buoy_5_t, buoy_5_rel_pos_valid, '--', color='blue')
    axis[0].set_title("RTK Fix buoy_5")
    axis[0].set_xlabel('timestamp')
    axis[0].set_ylabel('gnss fix')
    axis[0].grid()

    axis[1].plot(buoy_2_t, buoy_2_gnss_fix_ok, color='orange')
    axis[1].plot(buoy_2_t, buoy_2_carr_soln, '*', color='red')
    axis[1].plot(buoy_2_t, buoy_2_rel_pos_valid, '--', color='blue')
    axis[1].set_title("RTK Fix buoy_5")
    axis[1].set_xlabel('timestamp')
    axis[1].set_ylabel('gnss fix')
    axis[1].grid()

    axis[2].plot(buoy_3_t, buoy_3_gnss_fix_ok, color='orange')
    axis[2].plot(buoy_3_t, buoy_3_carr_soln, '*', color='red')
    axis[2].plot(buoy_3_t, buoy_3_rel_pos_valid, '--', color='blue')
    axis[2].set_title("RTK Fix buoy_5")
    axis[2].set_xlabel('timestamp')
    axis[2].set_ylabel('gnss fix')
    axis[2].grid()

    axis[3].plot(t_gt, gt_gnss_fix_ok, color='orange')
    axis[3].plot(t_gt, gt_carr_soln, '*', color='red')
    axis[3].plot(t_gt, gt_rel_pos_valid, '--', color='blue')
    axis[3].set_title("RTK Fix buoy_5")
    axis[3].set_xlabel('timestamp')
    axis[3].set_ylabel('gnss fix')
    axis[3].grid()

    plt.legend()
    plt.show()


def main():
    reader = Reader(
        'reihum_modem_test_mit_state_estimator_vergleich_mit_chris_logs')
    check_rtk_fix(reader)


if __name__ == '__main__':
    main()
