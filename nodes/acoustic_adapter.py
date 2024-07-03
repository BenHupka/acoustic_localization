#!/usr/bin/env python3
"""
Adapter für state_estimator, sendet anchor_poses und acoustic_distances wie ahoi
"""
import rclpy
from std_msgs.msg import Float64
from hippo_msgs.msg import AnchorPose, RangeMeasurement, AnchorPoses
from rclpy.node import Node
from ahoi.modem.modem import Modem
from rclpy.duration import Duration
import numpy as np


class acoustic_adapter(Node):

    def __init__(self):
        super().__init__(node_name='acoustic_adapter')

        self.k = 4  # Anzahl Anker
        self.n = 4  # Anzahl voller Zyklen vor einer Positionsabfrage
        self.i = 0  # Anker dst
        self.j = 0  # Zählvariable
        self.anchor_poses_initialized = 0  # prüfen ob bereits Ankerpositionen initialisiert

        self.anchor_poses_msg: AnchorPoses
        self.distances_gt = np.zeros(self.k)

        self.anchor_pose_pub = self.create_publisher(AnchorPose,
                                                     'anchor_poses_acoustic',
                                                     qos_profile=1)

        self.acoustic_distance_pub = self.create_publisher(
            msg_type=RangeMeasurement, topic='acoustic_distance', qos_profile=1)

        self.anchor_poses_sub = self.create_subscription(
            msg_type=AnchorPoses,
            topic='anchor_poses',
            callback=self.on_anchor_position,
            qos_profile=1)

        ################################## bisher ground truth nicht UWR ############################################
        self.anchor_0_distance_ground_truth_sub = self.create_subscription(
            msg_type=Float64,
            topic=
            'acoustic_distances_ground_truth/debug/anchor_1/distance',  # Anker id aus UWRange einen höher
            callback=self.on_distance_0_ground_truth,
            qos_profile=1)

        self.anchor_1_distance_ground_truth_sub = self.create_subscription(
            msg_type=Float64,
            topic=
            'acoustic_distances_ground_truth/debug/anchor_2/distance',  # Anker id aus UWRange einen höher
            callback=self.on_distance_1_ground_truth,
            qos_profile=1)

        self.anchor_2_distance_ground_truth_sub = self.create_subscription(
            msg_type=Float64,
            topic=
            'acoustic_distances_ground_truth/debug/anchor_3/distance',  # Anker id aus UWRange einen höher
            callback=self.on_distance_2_ground_truth,
            qos_profile=1)

        self.anchor_3_distance_ground_truth_sub = self.create_subscription(
            msg_type=Float64,
            topic=
            'acoustic_distances_ground_truth/debug/anchor_4/distance',  # Anker id aus UWRange einen höher
            callback=self.on_distance_3_ground_truth,
            qos_profile=1)

        self.timer = self.create_timer(timer_period_sec=1,
                                       callback=self.on_timer)

    def on_anchor_position(self, msg: AnchorPoses):
        self.anchor_poses_msg = msg
        if self.anchor_poses_initialized == 0:
            self.init_positions()

    def init_positions(self):
        for m in range(self.k):
            self.publish_anchor_pose(m)
            self.get_clock().sleep_for(
                Duration(seconds=1))  # wait before sending next position poll
        self.anchor_poses_initialized = 1

    def on_distance_0_ground_truth(self, msg: Float64):
        self.distances_gt[0] = msg.data

    def on_distance_1_ground_truth(self, msg: Float64):
        self.distances_gt[1] = msg.data

    def on_distance_2_ground_truth(self, msg: Float64):
        self.distances_gt[2] = msg.data

    def on_distance_3_ground_truth(self, msg: Float64):
        self.distances_gt[3] = msg.data

    def on_timer(self) -> None:
        if self.anchor_poses_initialized == 1:
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
                self.publish_anchor_pose(self.i)
                if self.i < self.k - 1:
                    self.i += 1
                elif self.i == self.k - 1:
                    self.i = 0
                self.j = 0

    def publish_anchor_pose(self, m):
        single_anchor_msg: AnchorPose
        single_anchor_msg = self.anchor_poses_msg.anchors[m]
        src = single_anchor_msg.id - 1  # UWRange fängt für Anker Modems mit ID 1 an
        position_x = single_anchor_msg.pose.pose.position.x
        position_y = single_anchor_msg.pose.pose.position.y
        position_z = single_anchor_msg.pose.pose.position.z
        print(f"position of anchor {src}: x = {position_x}, y = {position_y}")

        msg = AnchorPose()
        msg.pose.header.stamp = self.get_clock().now().to_msg()
        msg.pose.header.frame_id = 'map'
        msg.id = src
        msg.pose.pose.position.x = position_x
        msg.pose.pose.position.y = position_y
        msg.pose.pose.position.z = position_z
        print(f'publish {msg}')
        self.anchor_pose_pub.publish(msg)

    def publish_distance(self, src, distance):
        msg = RangeMeasurement()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.id = src
        msg.range = distance
        self.acoustic_distance_pub.publish(msg)


def main():
    rclpy.init()
    node = acoustic_adapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
