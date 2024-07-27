#!/usr/bin/env python3

from reader import Reader
import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime, timezone
import math


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

    t_0 = t_gt[100]
    t_end = 180
    # timestamps in Sekunden und Start auf Null
    t_gt_norm = (t_gt - t_0)
    time_norm = (time - t_0)

    time, id, distance, time_norm = crop_data3(time, id, distance, time_norm,
                                               t_end)

    x_gt, y_gt, t_gt, t_gt_norm = crop_data3(x_gt, y_gt, t_gt, t_gt_norm, t_end)

    # ################################## !!!!!!!!!!!!!! Ideen für Korrektur RTK
    x_offset = 0
    y_offset = 0
    # x_offset = -2.3
    # y_offset = -1.4
    x_gt += x_offset
    y_gt += y_offset
    print(f'x offset = {x_offset}, y offset = {y_offset}')
    ############################################################################

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
    distance_pressure_sensor_hydrophone = 0.52
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

    # rtk distances interpolated linearily and mapped to timestamps of acoustic measurements
    distance_gt_0_at_ranging = np.interp(time_ac_0, t_gt, distance_gt_0)
    distance_gt_1_at_ranging = np.interp(time_ac_1, t_gt, distance_gt_1)
    distance_gt_2_at_ranging = np.interp(time_ac_2, t_gt, distance_gt_2)

    # distance errors
    distance_0_error = distance_ac_0 - distance_gt_0_at_ranging
    distance_1_error = distance_ac_1 - distance_gt_1_at_ranging
    distance_2_error = distance_ac_2 - distance_gt_2_at_ranging

    #RMS error
    n = len(distance_0_error)
    E = 0.0
    for i in range(n):
        E += distance_0_error[i]**2
    rms_1 = math.sqrt(E / n)  # man beachte die Zuordnung der Bojennummer = id+1
    print(f'RMS error 1 = {rms_1}')

    n = len(distance_1_error)
    E = 0.0
    for i in range(n):
        E += distance_1_error[i]**2
    rms_2 = math.sqrt(E / n)
    print(f'RMS error 2 = {rms_2}')

    n = len(distance_2_error)
    E = 0.0
    for i in range(n):
        E += distance_2_error[i]**2
    rms_3 = math.sqrt(E / n)
    print(f'RMS error 3 = {rms_3}')

    # timestamps in Sekunden und Start auf Null
    time_ac_0_norm = time_ac_0 - t_0
    time_ac_1_norm = time_ac_1 - t_0
    time_ac_2_norm = time_ac_2 - t_0

    ###########################################################################
    # Idee Korrektur finden
    ###########################################################################
    #distances_ctd_sos - distances_rtk
    # buoy rtk values and depths interpolated linearily and mapped to timestamps of tof measures
    buoy_1_x_points = np.interp(time_ac_0, buoy_5_t, buoy_5_x)
    buoy_1_y_points = np.interp(time_ac_0, buoy_5_t, buoy_5_y)
    buoy_2_x_points = np.interp(time_ac_1, buoy_2_t, buoy_2_x)
    buoy_2_y_points = np.interp(time_ac_1, buoy_2_t, buoy_2_y)
    buoy_3_x_points = np.interp(time_ac_2, buoy_3_t, buoy_3_x)
    buoy_3_y_points = np.interp(time_ac_2, buoy_3_t, buoy_3_y)

    # rov rtk values and depths interpolated linearily and mapped to timestamps of tof measures
    rov_x_points_b1 = np.interp(time_ac_0, t_gt, x_gt)
    rov_y_points_b1 = np.interp(time_ac_0, t_gt, y_gt)
    rov_x_points_b2 = np.interp(time_ac_1, t_gt, x_gt)
    rov_y_points_b2 = np.interp(time_ac_1, t_gt, y_gt)
    rov_x_points_b3 = np.interp(time_ac_2, t_gt, x_gt)
    rov_y_points_b3 = np.interp(time_ac_2, t_gt, y_gt)

    # Kreis um Boje mit akustischer Distanz als Radius
    laufvariable = np.linspace(0, 2 * math.pi, 200)

    kreis_dict_x_b1 = {}
    kreis_dict_y_b1 = {}
    for i in range(len(time_ac_0)):
        kreis_dict_x_b1["kreis_{0}_x_b1".format(
            i)] = buoy_1_x_points[i] + distance_ac_0[i] * np.cos(laufvariable)
        kreis_dict_y_b1["kreis_{0}_y_b1".format(
            i)] = buoy_1_y_points[i] + distance_ac_0[i] * np.sin(laufvariable)
    kreis_dict_x_b1_keys = list(kreis_dict_x_b1.keys())
    kreis_dict_y_b1_keys = list(kreis_dict_y_b1.keys())

    kreis_dict_x_b2 = {}
    kreis_dict_y_b2 = {}
    for i in range(len(time_ac_1)):
        kreis_dict_x_b2["kreis_{0}_x_b2".format(
            i)] = buoy_2_x_points[i] + distance_ac_1[i] * np.cos(laufvariable)
        kreis_dict_y_b2["kreis_{0}_y_b2".format(
            i)] = buoy_2_y_points[i] + distance_ac_1[i] * np.sin(laufvariable)
    kreis_dict_x_b2_keys = list(kreis_dict_x_b2.keys())
    kreis_dict_y_b2_keys = list(kreis_dict_y_b2.keys())

    kreis_dict_x_b3 = {}
    kreis_dict_y_b3 = {}
    for i in range(len(time_ac_2)):
        kreis_dict_x_b3["kreis_{0}_x_b3".format(
            i)] = buoy_3_x_points[i] + distance_ac_2[i] * np.cos(laufvariable)
        kreis_dict_y_b3["kreis_{0}_y_b3".format(
            i)] = buoy_3_y_points[i] + distance_ac_2[i] * np.sin(laufvariable)
    kreis_dict_x_b3_keys = list(kreis_dict_x_b3.keys())
    kreis_dict_y_b3_keys = list(kreis_dict_y_b3.keys())

    # colors by t value

    def colorFader(
        c1,
        c2,
        mix=0
    ):  #fade (linear interpolate) from color c1 (at mix=0) to c2 (mix=1)
        c1 = np.array(mpl.colors.to_rgb(c1))
        c2 = np.array(mpl.colors.to_rgb(c2))
        return mpl.colors.to_hex((1 - mix) * c1 + mix * c2)

    c1 = 'whitesmoke'
    c2 = 'green'

    # position_error = np.sqrt(
    #     np.power(x_se - x_gt_norm, 2) + np.power(y_se - y_gt_norm, 2))

    # # CDF
    # # sort the data:
    # position_error_sorted = np.sort(position_error)
    # # calculate the proportional values of samples
    # p = 1. * np.arange(len(position_error)) / (len(position_error) - 1)

    ####################################################### export ###########
    # compress
    t_gt_norm_compressed = np.linspace(0, 180, 180)
    distance_gt_0_compressed = np.interp(t_gt_norm_compressed, t_gt_norm,
                                         distance_gt_0)
    distance_gt_1_compressed = np.interp(t_gt_norm_compressed, t_gt_norm,
                                         distance_gt_1)
    distance_gt_2_compressed = np.interp(t_gt_norm_compressed, t_gt_norm,
                                         distance_gt_2)

    # # acoustic distances
    # data = np.hstack(
    #     [time_ac_0_norm.reshape(-1, 1),
    #      distance_ac_0.reshape(-1, 1)])
    # np.savetxt('export/reihum_acoustic_distances_b1.csv',
    #            data,
    #            delimiter=',',
    #            header='timestamps_norm, distances_ctd',
    #            comments='')
    # data = np.hstack(
    #     [time_ac_1_norm.reshape(-1, 1),
    #      distance_ac_1.reshape(-1, 1)])
    # np.savetxt('export/reihum_acoustic_distances_b2.csv',
    #            data,
    #            delimiter=',',
    #            header='timestamps_norm, distances_ctd',
    #            comments='')
    # data = np.hstack(
    #     [time_ac_2_norm.reshape(-1, 1),
    #      distance_ac_2.reshape(-1, 1)])
    # np.savetxt('export/reihum_acoustic_distances_b3.csv',
    #            data,
    #            delimiter=',',
    #            header='timestamps_norm, distances_ctd',
    #            comments='')

    # # rtk distances
    # data = np.hstack([
    #     t_gt_norm_compressed.reshape(-1, 1),
    #     distance_gt_0_compressed.reshape(-1, 1)
    # ])
    # np.savetxt('export/reihum_rtk_distances_b1.csv',
    #            data,
    #            delimiter=',',
    #            header='t_gt_norm, distance_gt',
    #            comments='')
    # data = np.hstack([
    #     t_gt_norm_compressed.reshape(-1, 1),
    #     distance_gt_1_compressed.reshape(-1, 1)
    # ])
    # np.savetxt('export/reihum_rtk_distances_b2.csv',
    #            data,
    #            delimiter=',',
    #            header='t_gt_norm, distance_gt',
    #            comments='')
    # data = np.hstack([
    #     t_gt_norm_compressed.reshape(-1, 1),
    #     distance_gt_2_compressed.reshape(-1, 1)
    # ])
    # np.savetxt('export/reihum_rtk_distances.csv',
    #            data,
    #            delimiter=',',
    #            header='t_gt_norm, distance_gt',
    #            comments='')
    ##########################################################################

    ######################################### plots ###########################################
    # Kreise zur Korrektur
    plt.figure(1)

    for i, k in enumerate(kreis_dict_x_b1.keys()):
        plt.plot(kreis_dict_x_b1[k],
                 kreis_dict_y_b1[kreis_dict_y_b1_keys[i]],
                 color=colorFader(c1, c2, time_ac_0_norm[i] / t_gt_norm[-1]))
        plt.scatter(rov_x_points_b1[i],
                    rov_y_points_b1[i],
                    color=colorFader(c1, c2, time_ac_0_norm[i] / t_gt_norm[-1]),
                    s=30)
        plt.scatter(
            buoy_1_x_points[i],
            buoy_1_y_points[i],
        )

    for i, k in enumerate(kreis_dict_x_b2.keys()):
        plt.plot(kreis_dict_x_b2[k],
                 kreis_dict_y_b2[kreis_dict_y_b2_keys[i]],
                 color=colorFader(c1, c2, time_ac_1_norm[i] / t_gt_norm[-1]))
        plt.scatter(rov_x_points_b2[i],
                    rov_y_points_b2[i],
                    color=colorFader(c1, c2, time_ac_1_norm[i] / t_gt_norm[-1]),
                    s=30)
        plt.scatter(
            buoy_2_x_points[i],
            buoy_2_y_points[i],
        )

    for i, k in enumerate(kreis_dict_x_b3.keys()):
        plt.plot(kreis_dict_x_b3[k],
                 kreis_dict_y_b3[kreis_dict_y_b3_keys[i]],
                 color=colorFader(c1, c2, time_ac_2_norm[i] / t_gt_norm[-1]))
        plt.scatter(rov_x_points_b3[i],
                    rov_y_points_b3[i],
                    color=colorFader(c1, c2, time_ac_2_norm[i] / t_gt_norm[-1]),
                    s=30)
        plt.scatter(
            buoy_3_x_points[i],
            buoy_3_y_points[i],
        )

    plt.xlabel('east')
    plt.ylabel('north')
    plt.title('RTK Tracks')
    plt.axis('equal')  # für gleiche Skalierung
    plt.grid(True)
    plt.legend()
    plt.xlim(-20, -10)
    plt.ylim(-3, 6)
    plt.show()

    # RTK und akustische Distanzen zu allen 3 Bojen
    figure, axis = plt.subplots(3, 1)
    axis[0].plot(t_gt_norm_compressed, distance_gt_0_compressed, color='green')
    axis[0].scatter(time_ac_0_norm, distance_ac_0)
    axis[0].set_title("Distance buoy_1")
    axis[0].set_xlabel('timestamp')
    axis[0].set_ylabel('distance')
    axis[0].grid()

    axis[1].plot(t_gt_norm, distance_gt_1, color='green')
    axis[1].scatter(time_ac_1_norm, distance_ac_1)
    axis[1].set_title("Distance buoy_2")
    axis[1].set_xlabel('timestamp')
    axis[1].set_ylabel('distance')
    axis[1].grid()

    axis[2].plot(t_gt_norm, distance_gt_2, color='green')
    axis[2].scatter(time_ac_2_norm, distance_ac_2)
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
    # # axis[3].grid()

    plt.show()

    # # x und y von ROV und Boje 3
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
    axis[0].scatter(time_ac_0_norm, distance_0_error)
    axis[0].set_title("error buoy_1")
    axis[0].set_xlabel('timestamp')
    axis[0].set_ylabel('error')
    axis[0].grid()

    axis[1].scatter(time_ac_1_norm, distance_1_error)
    axis[1].set_title("error buoy_2")
    axis[1].set_xlabel('timestamp')
    axis[1].set_ylabel('error')
    axis[1].grid()

    axis[2].scatter(time_ac_2_norm, distance_2_error)
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
    reader = Reader(
        'reihum_modem_test_mit_state_estimator_vergleich_mit_chris_logs')
    plot_ground_truth_vs_estimate(reader)


if __name__ == '__main__':
    main()
