#!/usr/bin/env python3
"""
Zum Testen des state_estimators mit programmierten ground-truth-Pfaden
Adapter für state_estimator_test, sendet acoustic_distances wie ahoi, 
Ankerpositionen in state_estimator_test initialisieren, Rauschen Distanzmessungen
kann hier eingestellt werden, Rauschen Druckmessung in state_estimator_test
"""
import rclpy
from std_msgs.msg import Float64
from hippo_msgs.msg import AnchorPose, RangeMeasurement, AnchorPoses
from rclpy.node import Node
from ahoi.modem.modem import Modem
from rclpy.duration import Duration
import numpy as np
import math
from geometry_msgs.msg import PoseWithCovarianceStamped, Vector3Stamped


class ekf_test(Node):

    def __init__(self):
        super().__init__(node_name='ekf_test')

        self.start_time = self.get_clock().now()

        self.k = 4  # Anzahl Anker
        self.n = 4  # Anzahl voller Zyklen vor einer Positionsabfrage
        self.i = 0  # Anker dst
        self.j = 0  # Zählvariable

        self.gt = np.zeros(3)
        self.distances_gt = np.zeros(self.k)

        # anchor pose array, will be updatet with incoming position updates
        self.anchor_poses = np.zeros((self.k, 3))

        # initialize anchor_poses
        self.anchor_poses[0, :] = [0.0, 0.0, -1.0]
        self.anchor_poses[1, :] = [30.0, 0.0, -1.0]
        self.anchor_poses[2, :] = [30.0, 30.0, -1.0]
        self.anchor_poses[3, :] = [0.0, 30.0, -1.0]

        self.gt_pub = self.create_publisher(msg_type=Vector3Stamped,
                                            topic='ground_truth',
                                            qos_profile=1)

        self.acoustic_distance_pub = self.create_publisher(
            msg_type=RangeMeasurement, topic='acoustic_distance', qos_profile=1)

        self.gt_timer = self.create_timer(timer_period_sec=(1 / 50.0),
                                          callback=self.on_gt_timer)

        self.timer = self.create_timer(timer_period_sec=1,
                                       callback=self.on_timer)  #

    def on_gt_timer(self):
        now = self.get_clock().now()
        time = now - self.start_time
        self.gt[0] = 15.0 + 10.0 * math.cos(
            time.nanoseconds * 1e-9 * 2 * math.pi /
            (40 * math.pi))  # 0,5m/s Geschwindigkeit
        self.gt[1] = 15.0 + 10.0 * math.sin(
            time.nanoseconds * 1e-9 * 2 * math.pi / (40 * math.pi))
        self.gt[2] = -1.0

        for i in range(self.k):
            self.distances_gt[i] = math.sqrt(
                (self.anchor_poses[i, 0] - self.gt[0])**2 +
                (self.anchor_poses[i, 1] - self.gt[1])**2 +
                (self.anchor_poses[i, 2] - self.gt[2])**2)

        msg = Vector3Stamped()
        msg.header.stamp = now.to_msg()
        msg.vector.x = self.gt[0]
        msg.vector.y = self.gt[1]
        msg.vector.z = self.gt[2]
        self.gt_pub.publish(msg)

    def on_timer(self) -> None:
        # volle Zyklen vor Positionsabfrage
        if self.j < self.k * self.n:
            self.get_logger().info(f'send range poll to anchor {self.i}.')
            self.publish_distance(self.i, self.distances_gt[self.i])
            if self.i < self.k - 1:
                self.i += 1
            elif self.i == self.k - 1:
                self.i = 0
            self.j += 1
        # Poll mit anderem Pakettyp, fragt sowohl ranging als auch Position an
        elif self.j == self.k * self.n:
            self.get_logger().info(
                f'send range and position poll to anchor {self.i}.')
            self.publish_distance(self.i, self.distances_gt[self.i])
            self.j += 1
        # Warten auf Ankerposition
        elif self.j == self.k * self.n + 1:
            self.get_logger().info(f'wait for position of anchor {self.i}.')
            if self.i < self.k - 1:
                self.i += 1
            elif self.i == self.k - 1:
                self.i = 0
            self.j = 0

    def publish_distance(self, src, distance):
        msg = RangeMeasurement()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.id = src
        noise = np.zeros(1)
        # noise = np.random.normal(0, 0.1, 1) # add noise to range measurements
        self.get_logger().info(f'noise {noise[0]}.')
        msg.range = distance + noise[0]
        self.acoustic_distance_pub.publish(msg)


def main():
    rclpy.init()
    node = ekf_test()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
