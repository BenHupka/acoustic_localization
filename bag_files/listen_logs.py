#!/usr/bin/env python3

from reader import Reader
import matplotlib.pyplot as plt
import numpy as np
import json
from datetime import datetime, timezone


def prr_bojen(reader: Reader):
    # sent und received packets ROV
    sent_packets = reader.get_data('/bluerov01/sent_packets')
    n_messages = len(sent_packets)
    timestamps_sent = np.zeros([n_messages])
    src_sent = np.zeros([n_messages])
    dst_sent = np.zeros([n_messages])
    type_sent = np.zeros([n_messages])
    status_sent = np.zeros([n_messages])
    timestamps_sent_datetime = np.array([])

    i = 0
    for msg, time_received in sent_packets:
        timestamps_sent[i] = time_received * 1e-9
        src_sent[i] = msg.src
        dst_sent[i] = msg.dst
        type_sent[i] = msg.type
        status_sent[i] = msg.status
        timestamps_sent_datetime = np.append(
            timestamps_sent_datetime,
            datetime.fromtimestamp(timestamps_sent[i], timezone.utc))
        i += 1

    received_packets = reader.get_data('/bluerov01/received_packets')
    n_messages = len(received_packets)
    timestamps_received = np.zeros([n_messages])
    src_received = np.zeros([n_messages])
    dst_received = np.zeros([n_messages])
    type_received = np.zeros([n_messages])
    status_received = np.zeros([n_messages])
    timestamps_received_datetime = np.array([])

    i = 0
    for msg, time_received in received_packets:
        timestamps_received[i] = time_received * 1e-9
        src_received[i] = msg.src
        dst_received[i] = msg.dst
        type_received[i] = msg.type
        status_received[i] = msg.status
        timestamps_received_datetime = np.append(
            timestamps_received_datetime,
            datetime.fromtimestamp(timestamps_received[i], timezone.utc))
        i += 1

    # get sent and received packets to/from each buoy
    filter_arr_sent_0 = filter_id(0, dst_sent)
    dst_sent_0 = dst_sent[filter_arr_sent_0]
    timestamps_sent_0 = timestamps_sent[filter_arr_sent_0]
    timestamps_sent_datetime_0 = timestamps_sent_datetime[filter_arr_sent_0]
    filter_arr_sent_1 = filter_id(1, dst_sent)
    dst_sent_1 = dst_sent[filter_arr_sent_1]
    timestamps_sent_1 = timestamps_sent[filter_arr_sent_1]
    timestamps_sent_datetime_1 = timestamps_sent_datetime[filter_arr_sent_1]
    filter_arr_sent_2 = filter_id(2, dst_sent)
    dst_sent_2 = dst_sent[filter_arr_sent_2]
    timestamps_sent_2 = timestamps_sent[filter_arr_sent_2]
    timestamps_sent_datetime_2 = timestamps_sent_datetime[filter_arr_sent_2]

    filter_arr_received_0 = filter_id(0, src_received)
    src_received_0 = src_received[filter_arr_received_0]
    timestamps_received_0 = timestamps_received[filter_arr_received_0]
    timestamps_received_datetime_0 = timestamps_received_datetime[
        filter_arr_received_0]
    filter_arr_received_1 = filter_id(1, src_received)
    src_received_1 = src_received[filter_arr_received_1]
    timestamps_received_1 = timestamps_received[filter_arr_received_1]
    timestamps_received_datetime_1 = timestamps_received_datetime[
        filter_arr_received_1]
    filter_arr_received_2 = filter_id(2, src_received)
    src_received_2 = src_received[filter_arr_received_2]
    timestamps_received_2 = timestamps_received[filter_arr_received_2]
    timestamps_received_datetime_2 = timestamps_received_datetime[
        filter_arr_received_2]

    # logs boje 2 -> id 1
    f = open(
        '/home/ben/fav/ros2/src/acoustic_localization/bag_files/listen/buoy_2_listen_181227.json',
    )
    data = json.load(f)

    format_timestamp = "%Y-%m-%d %H:%M:%S.%f"
    epoch = datetime(1970, 1, 1, 0, 0, 0, 0)

    buoy_2_timestamps = np.zeros(len(data['packets']))
    buoy_2_datetime = np.array([])
    buoy_2_src = np.zeros(len(data['packets']))
    buoy_2_dst = np.zeros(len(data['packets']))
    buoy_2_type = np.zeros(len(data['packets']))
    buoy_2_status = np.zeros(len(data['packets']))

    i = 0
    for packet in data['packets']:
        date_time = "2024-07-04 " + packet['timestamp']
        datetime_object = datetime.strptime(date_time, format_timestamp)
        buoy_2_datetime = np.append(buoy_2_datetime, datetime_object)
        buoy_2_timestamps[i] = (datetime_object - epoch).total_seconds()
        buoy_2_src[i] = packet['header']['src']
        buoy_2_dst[i] = packet['header']['dst']
        buoy_2_type[i] = packet['header']['type']
        buoy_2_status[i] = packet['header']['status']
        i += 1

    # filter messages from ROV
    filter_buoy_2_from_rov = filter_id(9, buoy_2_src)
    buoy_2_timestamps_from_rov = buoy_2_timestamps[filter_buoy_2_from_rov]
    buoy_2_datetime_from_rov = buoy_2_datetime[filter_buoy_2_from_rov]
    buoy_2_src_from_rov = buoy_2_src[filter_buoy_2_from_rov]
    buoy_2_dst_from_rov = buoy_2_dst[filter_buoy_2_from_rov]
    buoy_2_type_from_rov = buoy_2_type[filter_buoy_2_from_rov]
    buoy_2_status_from_rov = buoy_2_status[filter_buoy_2_from_rov]

    #filter messages from ROV to buoy 2
    filter_buoy_2_from_rov_to_buoy = filter_id(29, buoy_2_dst_from_rov)
    buoy_2_timestamps_from_rov_to_buoy = buoy_2_timestamps_from_rov[
        filter_buoy_2_from_rov_to_buoy]
    buoy_2_datetime_from_rov_to_buoy = buoy_2_datetime_from_rov[
        filter_buoy_2_from_rov_to_buoy]
    buoy_2_src_from_rov_to_buoy = buoy_2_src_from_rov[
        filter_buoy_2_from_rov_to_buoy]
    buoy_2_dst_from_rov_to_buoy = buoy_2_dst_from_rov[
        filter_buoy_2_from_rov_to_buoy]
    buoy_2_type_from_rov_to_buoy = buoy_2_type_from_rov[
        filter_buoy_2_from_rov_to_buoy]
    buoy_2_status_from_rov_to_buoy = buoy_2_status_from_rov[
        filter_buoy_2_from_rov_to_buoy]

    plt.figure()
    plt.scatter(buoy_2_datetime_from_rov_to_buoy,
                buoy_2_dst_from_rov_to_buoy,
                color='green')
    plt.scatter(timestamps_sent_datetime_1, dst_sent_1, color='blue')
    plt.scatter(timestamps_received_datetime_1, src_received_1, color='orange')
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
    prr_bojen(reader)


if __name__ == '__main__':
    main()
