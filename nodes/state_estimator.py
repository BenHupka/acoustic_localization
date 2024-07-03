#!/usr/bin/env python3
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from hippo_msgs.msg import RangeMeasurement, AnchorPose
from rclpy.node import Node
from sensor_msgs.msg import FluidPressure

from tf_transformations import euler_from_quaternion
import math


class state_estimator(Node):

    def __init__(self):
        super().__init__(node_name='state_estimator')

        self.time_last_prediction = self.get_clock().now()

        # system specifications
        self.num_states = 6  # 3D Koordinaten des ROV Schwerpunkts, sowie Geschwindigkeiten
        self.num_anchors = 4
        self.distance_ROV_hydrophone = 0.0
        self.pressure = 101325.0

        # initial state
        self.x0 = np.array([[0.001], [0.001], [0.001], [0.0], [0.0], [0.0]])

        # estimated state, this will be updated in Kalman filter algorithm
        self.state = np.copy(self.x0)

        # initial state covariance - how sure are we about the state?
        # TODO initial state covariance is tuning knob
        # dimension: num states x num states
        # matrix needs to be positive definite and symmetric
        self.P0 = 1.0 * np.eye(self.num_states)

        # state covariance, this will be updated in Kalman filter algorithm
        self.P = self.P0

        # process noise covariance - how much noise do we add at each
        # prediction step?
        # TODO tuning knob
        # dimension: num states x num states
        # matrix needs to be positive definite and symmetric
        self.process_noise_position_stddev = 0.3
        self.Q = (self.process_noise_position_stddev**2) * np.eye(
            self.num_states)

        # measurement noise covariance - how much noise does the measurement
        # contain?
        # TODO tuning knob
        self.range_noise_stddev = 0.1
        # dimension: num measurements x num measurements
        # attention, this size is varying! -> Depends on detected Tags

        # anchor pose array, will be updatet with incoming position updates
        self.anchor_poses = np.zeros((self.num_anchors, 3))

        self.state_estimation_pub = self.create_publisher(
            msg_type=Odometry, topic='state_estimate', qos_profile=1)

        self.anchor_pose_sub = self.create_subscription(
            msg_type=AnchorPose,
            topic='anchor_poses_acoustic',
            callback=self.on_anchor_position,
            qos_profile=1)

        self.acoustic_distance_sub = self.create_subscription(
            msg_type=RangeMeasurement,
            topic='acoustic_distance',
            callback=self.on_acoustic_distance,
            qos_profile=1)

        #################################################################################
        self.pressure_sub = self.create_subscription(
            msg_type=FluidPressure,
            topic='pressure',
            callback=self.on_pressure,
            qos_profile=1)  # gerade nur so bei Simulation
        ##################################################################################

        # do prediction step with 50 Hz
        self.process_update_timer = self.create_timer(
            1 / 50, self.on_prediction_step_timer)

    def on_anchor_position(self, position_msg: AnchorPose):
        anchor_id = position_msg.id
        self.anchor_poses[anchor_id, 0] = position_msg.pose.pose.position.x
        self.anchor_poses[anchor_id, 1] = position_msg.pose.pose.position.y
        self.anchor_poses[anchor_id, 2] = position_msg.pose.pose.position.z
        self.get_logger().info(
            f'anker {anchor_id}: x={self.anchor_poses[anchor_id, 0]} y={self.anchor_poses[anchor_id, 1]} z={self.anchor_poses[anchor_id, 2]}'
        )

    def on_acoustic_distance(self, acoustic_distance: RangeMeasurement) -> None:
        anchor_id = acoustic_distance.id
        distance = acoustic_distance.range

        # before the measurement update, let's do a process update
        now = self.get_clock().now()
        dt = (now - self.time_last_prediction).nanoseconds * 1e-9
        self.prediction(dt)
        self.time_last_prediction = now

        self.measurement_update(anchor_id, distance)

    def on_pressure(self, pressure_msg: FluidPressure):
        self.pressure = pressure_msg.fluid_pressure

    def on_prediction_step_timer(self):
        # We will do a prediction step with a constant rate
        now = self.get_clock().now()
        dt = (now - self.time_last_prediction).nanoseconds * 1e-9
        self.prediction(dt)
        self.time_last_prediction = now

        # publish the estimated pose with constant rate
        self.publish_pose_msg(state=np.copy(self.state), now=now)

    def measurement_update(self, anchor_id, distance):
        num_measurements = 2  # eine Distanz und Druck
        measurements = np.array([[distance], [self.pressure]])
        h = np.zeros((num_measurements, 1))
        H = np.zeros((num_measurements, self.num_states))
        h[0, 0] = math.sqrt(
            (self.anchor_poses[anchor_id, 0] - self.state[0])**2 +
            (self.anchor_poses[anchor_id, 1] - self.state[1])**2 +
            (self.anchor_poses[anchor_id, 2] + self.distance_ROV_hydrophone -
             self.state[2])**2)  # Distanz zu Anker bei position estimate
        h[1, 0] = 101325.0 - 1e4 * self.state[
            2]  # Druckmessung bei position estimate
        H[0,
          0] = 1 / h[0, 0] * (self.state[0] - self.anchor_poses[anchor_id, 0])
        H[0,
          1] = 1 / h[0, 0] * (self.state[1] - self.anchor_poses[anchor_id, 1])
        H[0, 2] = 1 / h[0, 0] * (self.state[2] - self.distance_ROV_hydrophone -
                                 self.anchor_poses[anchor_id, 2])
        H[1, 2] = -1e4

        y = measurements - h
        R = np.eye(num_measurements) * (self.range_noise_stddev**2)
        S = H @ self.P @ H.transpose() + R
        K = self.P @ H.transpose() @ np.linalg.inv(S)
        self.state = self.state + K @ y
        self.P = (np.eye(self.num_states) - (K @ H)) @ self.P

    def prediction(self, dt: float):
        f_jacobian = np.array([[1, 0, 0, dt, 0, 0], [0, 1, 0, 0, dt, 0],
                               [0, 0, 1, 0, 0, dt], [0, 0, 0, 1, 0, 0],
                               [0, 0, 0, 0, 1, 0], [0, 0, 0, 0, 0, 1]])
        self.state = f_jacobian @ self.state
        self.P = f_jacobian @ self.P @ f_jacobian.transpose() + self.Q

    def publish_pose_msg(self, state: np.ndarray, now: rclpy.time.Time) -> None:
        msg = Odometry()

        msg.header.stamp = now.to_msg()
        msg.header.frame_id = "map"
        msg.child_frame_id = "map"
        msg.pose.pose.position.x = state[0, 0]
        msg.pose.pose.position.y = state[1, 0]
        msg.pose.pose.position.z = state[2, 0]
        msg.twist.twist.linear.x = state[3, 0]
        msg.twist.twist.linear.y = state[4, 0]
        msg.twist.twist.linear.z = state[5, 0]

        self.state_estimation_pub.publish(msg)


def main():
    rclpy.init()
    node = state_estimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
