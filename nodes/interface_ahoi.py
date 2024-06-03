#!/usr/bin/env python3
"""
Test für Interface zu Ahoi Modem
"""
import rclpy
import math
from hippo_msgs.msg import AnchorPose, RangeMeasurement, AhoiPacket
from rclpy.node import Node
from ahoi.modem.modem import Modem
from rclpy.duration import Duration


class interface_ahoi(Node):

    def __init__(self):
        super().__init__(node_name='interface_ahoi')

        self.k = 1  # Anzahl Anker
        self.n = 1  # Anzahl voller Zyklen vor einer Positionsabfrage
        self.i = 0  # Anker dst
        self.j = 0  # Zählvariable

        self.initial_position_bool = [
            False
        ] * self.k  # initiale Positionsabfrage erfolgreich?

        self.myModem = Modem()
        self.myModem.connect("/dev/ttyUSB0")
        self.myModem.setTxEcho(
            True)  # übertragene und empfangene Pakete in Terminal echoen
        self.myModem.setRxEcho(True)

        self.myModem.addRxCallback(self.rangingCallback)
        self.myModem.receive(thread=True)

        self.anchor_pose_pub = self.create_publisher(AnchorPose,
                                                     'anchor_poses_acoustic',
                                                     qos_profile=1)

        self.acoustic_distance_pub = self.create_publisher(
            msg_type=RangeMeasurement, topic='acoustic_distance', qos_profile=1)

        self.sent_packets_pub = self.create_publisher(msg_type=AhoiPacket,
                                                      topic='sent_packets',
                                                      qos_profile=1)

        self.received_packets_pub = self.create_publisher(
            msg_type=AhoiPacket, topic='received_packets', qos_profile=1)

        self.init_positions()

        self.timer = self.create_timer(timer_period_sec=1,
                                       callback=self.on_timer)

    def init_positions(self):
        for m, bool in enumerate(self.initial_position_bool):
            while not bool:  # bool wird leider nicht aktualisiert, wenn Position angekommen
                if self.initial_position_bool[
                        m] == True:  # hier explizit bool abfragen, der dann aktualisierter Wert ist
                    self.get_logger().info(f'initial position {m} successful')
                    break
                self.get_logger().info(f'pinging anchor {m}')
                self.myModem.send(dst=m,
                                  payload=bytearray(),
                                  status=0,
                                  src=9,
                                  type=0x7C)
                self.publish_sent_packets(9, m, 124, 0)
                self.get_clock().sleep_for(Duration(
                    seconds=1))  # wait before sending next position poll

    def on_timer(self) -> None:
        # volle Zyklen vor Positionsabfrage
        if self.j < self.k * self.n:
            self.get_logger().info(f'send range poll to anchor {self.i}.')
            self.myModem.send(dst=self.i,
                              payload=bytearray(),
                              status=2,
                              src=9,
                              type=0)
            self.publish_sent_packets(9, self.i, 0, 2)
            if self.i < self.k - 1:
                self.i += 1
            elif self.i == self.k - 1:
                self.i = 0
            self.j += 1
        # Poll mit anderem Pakettyp, fragt sowohl ranging als auch Position an
        elif self.j == self.k * self.n:
            self.get_logger().info(
                f'send range and position poll to anchor {self.i}.')
            self.myModem.send(dst=self.i,
                              payload=bytearray(),
                              status=2,
                              src=9,
                              type=0x7E)
            self.publish_sent_packets(9, self.i, 126, 2)
            self.j += 1
        # Warten auf Ankerposition
        elif self.j == self.k * self.n + 1:
            self.get_logger().info(f'wait for position of anchor {self.i}.')
            if self.i < self.k - 1:
                self.i += 1
            elif self.i == self.k - 1:
                self.i = 0
            self.j = 0

    def rangingCallback(self, pkt):
        SPEED_OF_SOUND = 1500  # in meters per second
        # check if we have received a ranging ack
        if pkt.header.type == 0x7F and pkt.header.len > 0:
            src = pkt.header.src
            dst = pkt.header.dst
            type = pkt.header.type
            status = pkt.header.status
            self.publish_received_packets(src, dst, type, status)

            tof = 0
            for i in range(0, 4):
                tof = tof * 256 + pkt.payload[i]
            distance = tof * 1e-6 * SPEED_OF_SOUND
            self.get_logger().info(f"distance to {src}: %6.1f" % (distance))

            self.publish_distance(src, distance)

        # check if we have received position update
        if pkt.header.type == 0x7D and pkt.header.len > 0:
            src = pkt.header.src
            dst = pkt.header.dst
            type = pkt.header.type
            status = pkt.header.status
            self.publish_received_packets(src, dst, type, status)

            position_x = int.from_bytes(pkt.payload[0:2], 'big',
                                        signed=True) * 1e-2
            position_y = int.from_bytes(pkt.payload[2:4], 'big',
                                        signed=True) * 1e-2
            self.get_logger().info(
                f"position of anchor {src}: x = {position_x}, y = {position_y}")

            self.publish_anchor_pose(src, position_x, position_y)

        # check if we have received initial position
        if pkt.header.type == 0x7B and pkt.header.len > 0:
            src = pkt.header.src
            dst = pkt.header.dst
            type = pkt.header.type
            status = pkt.header.status
            self.publish_received_packets(src, dst, type, status)

            position_x = int.from_bytes(pkt.payload[0:2], 'big',
                                        signed=True) * 1e-2
            position_y = int.from_bytes(pkt.payload[2:4], 'big',
                                        signed=True) * 1e-2
            self.get_logger().info(
                f"initial position of anchor {src}: x = {position_x}, y = {position_y}"
            )
            self.initial_position_bool[src] = True

            self.publish_anchor_pose(src, position_x, position_y)

    def publish_anchor_pose(self, src, position_x, position_y):
        msg = AnchorPose()
        msg.pose.header.stamp = self.get_clock().now().to_msg()
        msg.pose.header.frame_id = 'map'
        msg.id = src
        msg.pose.pose.position.x = position_x
        msg.pose.pose.position.y = position_y
        msg.pose.pose.position.z = -1.0
        self.anchor_pose_pub.publish(msg)

    def publish_distance(self, src, distance):
        msg = RangeMeasurement()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.id = src
        msg.range = distance
        self.acoustic_distance_pub.publish(msg)

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


def main():
    rclpy.init()
    node = interface_ahoi()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
