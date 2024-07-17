#!/usr/bin/env python3

from reader import Reader
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import curve_fit


def prr(reader: Reader):
    sent_packets = reader.get_data('/bluerov01/sent_packets')
    n_messages = len(sent_packets)
    timestamps_sent = np.zeros([n_messages])
    src_sent = np.zeros([n_messages])
    dst_sent = np.zeros([n_messages])
    type_sent = np.zeros([n_messages])
    status_sent = np.zeros([n_messages])

    i = 0
    for msg, time_received in sent_packets:
        timestamps_sent[i] = time_received * 1e-9
        src_sent[i] = msg.src
        dst_sent[i] = msg.dst
        type_sent[i] = msg.type
        status_sent[i] = msg.status
        i += 1

    received_packets = reader.get_data('/bluerov01/received_packets')
    n_messages = len(received_packets)
    timestamps_received = np.zeros([n_messages])
    src_received = np.zeros([n_messages])
    dst_received = np.zeros([n_messages])
    type_received = np.zeros([n_messages])
    status_received = np.zeros([n_messages])

    i = 0
    for msg, time_received in received_packets:
        timestamps_received[i] = time_received * 1e-9
        src_received[i] = msg.src
        dst_received[i] = msg.dst
        type_received[i] = msg.type
        status_received[i] = msg.status
        i += 1

    ground_truth_data = reader.get_data('/bluerov01/vehicle')
    n_messages = len(ground_truth_data)
    x_gt = np.zeros([n_messages])
    y_gt = np.zeros([n_messages])
    t_gt = np.zeros([n_messages])

    i = 0
    for msg, time_received in ground_truth_data:
        x_gt[i] = msg.east
        y_gt[i] = msg.north
        t_gt[i] = time_received * 1e-9
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

    # calculated hydrophone depth
    pressure_surface = 100000.0
    density_water = 1e4
    distance_pressure_sensor_hydrophone = -0.25
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
    buoy_hydrophone_depth_norm = 1.55 * np.ones(len(ROV_hydrophone_depth_norm))

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

    # get sent and received packets to/from each buoy
    filter_arr_sent_0 = filter_id(0, dst_sent)
    dst_sent_0 = dst_sent[filter_arr_sent_0]
    time_sent_0 = timestamps_sent[filter_arr_sent_0]
    filter_arr_sent_1 = filter_id(1, dst_sent)
    dst_sent_1 = dst_sent[filter_arr_sent_1]
    time_sent_1 = timestamps_sent[filter_arr_sent_1]
    filter_arr_sent_2 = filter_id(2, dst_sent)
    dst_sent_2 = dst_sent[filter_arr_sent_2]
    time_sent_2 = timestamps_sent[filter_arr_sent_2]

    filter_arr_received_0 = filter_id(0, src_received)
    src_received_0 = src_received[filter_arr_received_0]
    time_received_0 = timestamps_received[filter_arr_received_0]
    filter_arr_received_1 = filter_id(1, src_received)
    src_received_1 = src_received[filter_arr_received_1]
    time_received_1 = timestamps_received[filter_arr_received_1]
    filter_arr_received_2 = filter_id(2, src_received)
    src_received_2 = src_received[filter_arr_received_2]
    time_received_2 = timestamps_received[filter_arr_received_2]

    # get distance at timestamp of received packet
    distance_gt_0_received = np.interp(time_received_0, t_gt, distance_gt_0)
    distance_gt_1_received = np.interp(time_received_1, t_gt, distance_gt_1)
    distance_gt_2_received = np.interp(time_received_2, t_gt, distance_gt_2)

    # packet reception rates (full cycle)
    prr_0 = len(src_received_0) / len(dst_sent_0)
    prr_1 = len(src_received_1) / len(dst_sent_1)
    prr_2 = len(src_received_2) / len(dst_sent_2)
    print(f'prr_0 = {prr_0}, prr_1 = {prr_1}, prr_2 = {prr_2}')

    # time passed since last distance update
    time_since_last_ack_0 = np.zeros(len(src_received_0))
    for i, src in enumerate(src_received_0):
        if i == 0:
            time_since_last_ack_0[i] = 0.0
        else:
            time_since_last_ack_0[i] = time_received_0[i] - time_received_0[i -
                                                                            1]
    time_since_last_ack_1 = np.zeros(len(src_received_1))
    for i, src in enumerate(src_received_1):
        if i == 0:
            time_since_last_ack_1[i] = 0.0
        else:
            time_since_last_ack_1[i] = time_received_1[i] - time_received_1[i -
                                                                            1]
    time_since_last_ack_2 = np.zeros(len(src_received_2))
    for i, src in enumerate(src_received_2):
        if i == 0:
            time_since_last_ack_2[i] = 0.0
        else:
            time_since_last_ack_2[i] = time_received_2[i] - time_received_2[i -
                                                                            1]

    # sent and received packets per buoy
    figure, axis = plt.subplots(3, 1)
    axis[0].scatter(time_sent_0,
                    dst_sent_0,
                    color='blue',
                    label='sent packages')
    axis[0].scatter(time_received_0,
                    src_received_0,
                    color='red',
                    label='received packages')
    axis[0].scatter(time_received_0,
                    time_since_last_ack_0,
                    color='green',
                    label='time since last ack')
    axis[0].set_title("packets to/from buoy 5 (1)")
    axis[0].set_xlabel('timestamp')
    axis[0].set_ylabel('id')
    axis[0].grid()

    axis[1].scatter(time_sent_1,
                    dst_sent_1,
                    color='blue',
                    label='sent packages')
    axis[1].scatter(time_received_1,
                    src_received_1,
                    color='red',
                    label='received packages')
    axis[1].scatter(time_received_1,
                    time_since_last_ack_1,
                    color='green',
                    label='time since last ack')
    axis[1].set_title("packets to/from buoy 2")
    axis[1].set_xlabel('timestamp')
    axis[1].set_ylabel('id')
    axis[1].grid()

    axis[2].scatter(time_sent_2,
                    dst_sent_2,
                    color='blue',
                    label='sent packages')
    axis[2].scatter(time_received_2,
                    src_received_2,
                    color='red',
                    label='received packages')
    axis[2].scatter(time_received_2,
                    time_since_last_ack_2,
                    color='green',
                    label='time since last ack')
    axis[2].set_title("packets to/from buoy 3")
    axis[2].set_xlabel('timestamp')
    axis[2].set_ylabel('id')
    axis[2].grid()

    plt.legend()
    plt.show()

    # time since last ack over distance
    figure, axis = plt.subplots(3, 1)
    axis[0].scatter(distance_gt_0_received, time_since_last_ack_0)
    axis[0].set_title("packets to/from buoy 5 (1)")
    axis[0].set_xlabel('rtk distance')
    axis[0].set_ylabel('time since last ack')
    axis[0].grid()

    axis[1].scatter(distance_gt_1_received, time_since_last_ack_1)
    axis[1].set_title("packets to/from buoy 2")
    axis[1].set_xlabel('rtk distance')
    axis[1].set_ylabel('time since last ack')
    axis[1].grid()

    axis[2].scatter(distance_gt_2_received, time_since_last_ack_2)
    axis[2].set_title("packets to/from buoy 3")
    axis[2].set_xlabel('rtk distance')
    axis[2].set_ylabel('time since last ack')
    axis[2].grid()

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
        'reihum_modem_test_mit_state_estimator_vergleich_mit_chris_logs'
    )  #reihum_modem_test_mit_state_estimator_vergleich_mit_chris_logs
    prr(reader)


if __name__ == '__main__':
    main()
