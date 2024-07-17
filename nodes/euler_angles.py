#!/usr/bin/env python3
"""

"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from tf_transformations import euler_from_quaternion
from geometry_msgs.msg import PoseWithCovarianceStamped, Vector3Stamped
from px4_msgs.msg import VehicleOdometry


class euler(Node):

    def __init__(self):
        super().__init__(node_name='euler')

        qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         history=QoSHistoryPolicy.KEEP_LAST,
                         depth=1)

        self.euler_pub = self.create_publisher(msg_type=Vector3Stamped,
                                               topic='depth',
                                               qos_profile=1)
        self.quaternion_sub = self.create_subscription(
            msg_type=VehicleOdometry,
            topic='/bluerov01/fmu/out/vehicle_odometry',
            callback=self.on_quaternion,
            qos_profile=qos)

    def on_quaternion(self, msg: VehicleOdometry):
        # get the vehicle orientation expressed as quaternion
        q = msg.q
        stamp = msg.timestamp
        seconds = int(stamp * 10**(-6))
        nanoseconds = (stamp - seconds * 10**6) * 10**3
        # convert the quaternion to euler angles
        (roll, pitch, yaw) = euler_from_quaternion([q[0], q[1], q[2], q[3]])

        msg = Vector3Stamped()
        msg.header.stamp.sec = seconds
        msg.header.stamp.nanosec = nanoseconds
        msg.vector.x = roll
        msg.vector.y = pitch
        msg.vector.z = yaw
        self.euler_pub.publish(msg)


def main():
    rclpy.init()
    node = euler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
