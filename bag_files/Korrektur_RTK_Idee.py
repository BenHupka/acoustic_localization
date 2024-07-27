#!/usr/bin/env python3

from reader import Reader
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import curve_fit
from datetime import datetime, timezone
import math
import matplotlib as mpl


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

    t_0 = t_gt[0]

    # ################################## !!!!!!!!!!!!!! Ideen für Korrektur RTK
    x_gt -= -1
    y_gt -= 0.25

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
    distance_pressure_sensor_hydrophone = -0.15  # bei two_modems_dynamic_v4 und reihum 0.52, sonst -0.15
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
    depth_difference = np.interp(
        timestamps, t_gt,
        buoy_hydrophone_depth_norm - ROV_hydrophone_depth_norm)

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

    # timestamps in Sekunden und Start auf Null
    t_gt_norm = (t_gt - t_0)
    timestamps_norm = (timestamps - t_0)

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

    ###########################################################################
    # Idee Korrektur finden
    ###########################################################################
    #distances_ctd_sos - distances_rtk
    # buoy rtk values and depths interpolated linearily and mapped to timestamps of tof measures
    buoy_2_x_points = np.interp(timestamps, buoy_2_t, buoy_2_x)
    buoy_2_y_points = np.interp(timestamps, buoy_2_t, buoy_2_y)

    # rov rtk values and depths interpolated linearily and mapped to timestamps of tof measures
    rov_x_points = np.interp(timestamps, t_gt, x_gt)
    rov_y_points = np.interp(timestamps, t_gt, y_gt)

    # Kreis um Boje mit akustischer Distanz als Radius
    laufvariable = np.linspace(0, 2 * math.pi, 100)
    # kreis_0_x = buoy_2_x_points[0] + distances_ctd_sos[0] * np.cos(laufvariable)
    # kreis_0_y = buoy_2_y_points[0] + distances_ctd_sos[0] * np.sin(laufvariable)

    radius_projected = np.sqrt(
        np.power(distances_ctd_sos, 2) - np.power(depth_difference, 2))

    kreis_dict_x = {}
    kreis_dict_y = {}
    for i in range(15):
        kreis_dict_x["kreis_{0}_x".format(
            i
        )] = buoy_2_x_points[i] + radius_projected[i] * np.cos(laufvariable)
        kreis_dict_y["kreis_{0}_y".format(
            i
        )] = buoy_2_y_points[i] + radius_projected[i] * np.sin(laufvariable)
    kreis_dict_x_keys = list(kreis_dict_x.keys())
    kreis_dict_y_keys = list(kreis_dict_y.keys())

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

    ############ Korrektur in Zeile 51

    ###########################################################################

    plt.figure(1)

    for i, k in enumerate(kreis_dict_x.keys()):
        plt.plot(kreis_dict_x[k],
                 kreis_dict_y[kreis_dict_y_keys[i]],
                 color=colorFader(c1, c2,
                                  timestamps_norm[i] / timestamps_norm[-1]))
        plt.scatter(rov_x_points[i],
                    rov_y_points[i],
                    color=colorFader(c1, c2,
                                     timestamps_norm[i] / timestamps_norm[-1]),
                    s=20)
        plt.scatter(
            buoy_2_x_points[i],
            buoy_2_y_points[i],
        )

    plt.xlabel('east')
    plt.ylabel('north')
    plt.title('RTK Tracks')
    plt.axis('equal')  # für gleiche Skalierung
    plt.grid(True)
    plt.legend()
    plt.show()

    # plt.figure(2)

    # # Track
    # plt.plot(x_gt, y_gt, label='Ground Truth RTK')
    # # plt.plot(x_se, y_se, label='State Estimate')
    # # plt.plot(buoy_5_x, buoy_5_y,
    # #          label='buoy_1')  # Boje 1 als Bezeichnung für BA
    # plt.plot(buoy_2_x, buoy_2_y, label='buoy_2')
    # # plt.plot(buoy_3_x, buoy_3_y, label='buoy_3')
    # plt.xlabel('east')
    # plt.ylabel('north')
    # plt.title('RTK Tracks')
    # plt.axis('equal')  # für gleiche Skalierung

    # plt.grid(True)
    # plt.legend()
    # # plt.show()

    # figure, axis = plt.subplots(2, 1)
    # # axis[0].scatter(distances_rtk, tof)
    # # axis[0].plot(x_regr, y_regr, '--', color='red')
    # # axis[0].set_title("Best fit through tof over distance")
    # # axis[0].set_xlabel('distance')
    # # axis[0].set_ylabel('tof')
    # # axis[0].grid()

    # axis[0].scatter(timestamps_norm,
    #                 distances_ctd_sos,
    #                 label='acoustic distances')
    # # axis[1].plot(timestamps, distances_rtk, label='rtk distances ac')
    # axis[0].plot(t_gt_norm, distance_gt_1, label='rtk distances', color='green')
    # axis[0].set_title("Acoustic vs rtk distances")
    # axis[0].set_xlabel('timestamp')
    # axis[0].set_ylabel('distance')
    # axis[0].set_xlim([t_gt_norm[0], t_gt_norm[-1]])
    # axis[0].grid()

    # axis[1].scatter(timestamps_norm, error, label='error', color='red')
    # axis[1].set_title("ranging error")
    # axis[1].set_xlabel('timestamp')
    # axis[1].set_ylabel('error')
    # axis[1].set_xlim([t_gt_norm[0], t_gt_norm[-1]])
    # axis[1].grid()
    # plt.show()


def main():
    reader = Reader('two_modems_dynamic_v3'
                    )  # ab two_modems_dynamic_v4 andere hydrophone depth!!!
    calculate_sos(reader)


if __name__ == '__main__':
    main()
