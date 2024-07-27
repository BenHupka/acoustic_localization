#!/usr/bin/env python3
"""

"""
import rclpy
from hippo_msgs.msg import AhoiPacket, RTK, DistanceTOF
from rclpy.node import Node
from ahoi.modem.modem import Modem
from sensor_msgs.msg import FluidPressure
from std_msgs.msg import Byte
import math


class vehicle_corrected(Node):

    def __init__(self):
        super().__init__(node_name='vehicle_corrected')

        self.vehicle_pub_corrected = self.create_publisher(
            msg_type=RTK, topic='vehicle_corrected', qos_profile=1)
        self.vehicle_sub = self.create_subscription(msg_type=RTK,
                                                    topic='vehicle',
                                                    callback=self.on_vehicle,
                                                    qos_profile=1)

    def on_vehicle(self, vehicle_msg: RTK):
        self.vehicle_north = vehicle_msg.north
        self.vehicle_east = vehicle_msg.east
        self.stamp = vehicle_msg.header._stamp

        vehicle_corrected = RTK()
        vehicle_corrected.id = 9
        vehicle_corrected.header.stamp = self.stamp
        vehicle_corrected.north = self.vehicle_north - 1.4
        vehicle_corrected.east = self.vehicle_east - 2.3

        self.vehicle_pub_corrected.publish(vehicle_corrected)


def main():
    rclpy.init()
    node = vehicle_corrected()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
