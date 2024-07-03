#!/usr/bin/env python3
"""
Ermittlung der Präzision der TOF Messungen, nutzen zur Initialisierung der SOS
"""
import rclpy
from hippo_msgs.msg import AhoiPacket, RTK, DistanceTOF
from rclpy.node import Node
from ahoi.modem.modem import Modem
from sensor_msgs.msg import FluidPressure
from std_msgs.msg import Byte
import math


class init_sos(Node):

    def __init__(self):
        super().__init__(node_name='init_sos')

        self.buoy_id = 0
        self.buoy_1_north = 0.0
        self.buoy_1_east = 0.0
        self.buoy_1_depth = 1.0  # change depending on buoy setup
        self.vehicle_north = 0.0
        self.vehicle_east = 0.0
        self.distance_hydrophone_depth_sensor = 0.0  # change depending on ROV setup
        self.hydrophone_depth = self.distance_hydrophone_depth_sensor

        self.myModem = Modem()
        self.myModem.connect("/dev/ttyUSB1")
        self.myModem.setTxEcho(
            True)  # übertragene und empfangene Pakete in Terminal echoen
        self.myModem.setRxEcho(True)

        self.myModem.addRxCallback(self.rangingCallback)
        self.myModem.receive(thread=True)

        self.sent_packets_pub = self.create_publisher(msg_type=AhoiPacket,
                                                      topic='sent_packets',
                                                      qos_profile=1)

        self.received_packets_pub = self.create_publisher(
            msg_type=AhoiPacket, topic='received_packets', qos_profile=1)

        self.distance_tof_pub = self.create_publisher(msg_type=DistanceTOF,
                                                      topic='distance_tof',
                                                      qos_profile=1)

        self.measurement_start_sub = self.create_subscription(
            msg_type=Byte,
            topic='measurement_start',
            callback=self.on_measurement_start,
            qos_profile=1)

        self.buoy_1_sub = self.create_subscription(msg_type=RTK,
                                                   topic='buoy_1',
                                                   callback=self.on_buoy_1,
                                                   qos_profile=1)

        self.vehicle_sub = self.create_subscription(msg_type=RTK,
                                                    topic='vehicle',
                                                    callback=self.on_vehicle,
                                                    qos_profile=1)

        #################################################################################
        self.pressure_sub = self.create_subscription(
            msg_type=FluidPressure,
            topic='pressure',
            callback=self.on_pressure,
            qos_profile=1)  # gerade nur so bei Simulation
        ##################################################################################

    def on_measurement_start(self, msg: Byte):
        self.myModem.send(dst=self.buoy_id,
                          payload=bytearray(),
                          status=2,
                          src=9,
                          type=0)
        self.publish_sent_packets(9, self.buoy_id, 0, 2)

    def on_buoy_1(self, buoy_1_msg: RTK):
        self.buoy_1_north = buoy_1_msg.north
        self.buoy_1_east = buoy_1_msg.east

    def on_vehicle(self, vehicle_msg: RTK):
        self.vehicle_north = vehicle_msg.north
        self.vehicle_east = vehicle_msg.east

    def on_pressure(self, pressure_msg: FluidPressure):
        pressure = pressure_msg.fluid_pressure
        pressure_surface = 101325.0
        density_water = 1e4
        self.hydrophone_depth = (
            pressure - pressure_surface
        ) / density_water + self.distance_hydrophone_depth_sensor

    def rangingCallback(self, pkt):
        # check if we have received a ranging ack
        if pkt.header.type == 0x7F and pkt.header.len > 0:
            src = pkt.header.src
            dst = pkt.header.dst
            type = pkt.header.type
            status = pkt.header.status
            self.publish_received_packets(src, dst, type, status)

            # calculate tof
            tof_micros = 0
            for i in range(0, 4):
                tof_micros = tof_micros * 256 + pkt.payload[i]

            tof = tof_micros * 1e-6
            d_range = tof * 1500
            self.get_logger().info(f"tof: %6.2f" % (tof))
            self.get_logger().info(f"range estimate: %6.2f" % (d_range))

            # calculate RTK distance
            distance = math.sqrt(
                (self.buoy_1_north - self.vehicle_north)**2 +
                (self.buoy_1_east - self.vehicle_east)**2 +
                (self.buoy_1_depth - self.hydrophone_depth)**
                2)  # distance between hydrophones according to RTK and pressure
            self.get_logger().info(f"distance: %6.2f" % (distance))

            # publish tof over distance
            self.publish_distance_tof(tof, distance)

    def publish_sent_packets(self, src, dst, type, status):
        msg = AhoiPacket()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.src = src
        msg.dst = dst
        msg.type = type
        msg.status = status
        self.sent_packets_pub.publish(msg)

    def publish_received_packets(self, src, dst, type, status):
        msg = AhoiPacket()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.src = src
        msg.dst = dst
        msg.type = type
        msg.status = status
        self.received_packets_pub.publish(msg)

    def publish_distance_tof(self, tof, distance):
        msg = DistanceTOF()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.tof = tof
        msg.distance = distance
        self.distance_tof_pub.publish(msg)


def main():
    rclpy.init()
    node = init_sos()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
