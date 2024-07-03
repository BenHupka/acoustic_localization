#!/usr/bin/env python3
"""
Interface zu Ahoi Modem, Polling-Algorithmus, Publisher von Distanzen und akustisch übertragenen Ankerpositionen
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Byte


class measurement_start_timer(Node):

    def __init__(self):
        super().__init__(node_name='measurement_start_timer')

        self.measurement_start_pub = self.create_publisher(
            msg_type=Byte, topic='measurement_start', qos_profile=1)

        self.timer = self.create_timer(timer_period_sec=1,
                                       callback=self.on_timer)

    def on_timer(self) -> None:
        msg = Byte()
        self.measurement_start_pub.publish(msg)


def main():
    rclpy.init()
    node = measurement_start_timer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
