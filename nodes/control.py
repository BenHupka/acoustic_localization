#!/usr/bin/env python3
"""
This node is your xyz controller.
It takes as input a current xyz and a given xyz setpoint.
Its output is a thrust command to the BlueROV's actuators.
"""
import rclpy
from hippo_msgs.msg import Float64Stamped
from nav_msgs.msg import Odometry
from hippo_control_msgs.msg import ActuatorSetpoint
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PointStamped
from dvl_msgs.msg import DeadReckoningReport
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
import math
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from tf_transformations import euler_from_quaternion
import numpy as np

time = 0
previous_time = 0
integral_x = 0
previous_error_x = 0
integral_y = 0
previous_error_y = 0
integral_z = 0
previous_error_z = 0


class control(Node):

    def __init__(self):
        super().__init__(node_name='control')
        qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         history=QoSHistoryPolicy.KEEP_LAST,
                         depth=1)

        # default value for yaw
        self.yaw = math.pi / 2.0

        # create setpoints for path
        self.setpoints = np.zeros((200, 3))
        for i in range(200):
            x_start = 0.0
            y_start = 0.0
            if i < 50:
                self.setpoints[i, 0] = x_start + i * 0.1
                self.setpoints[i, 1] = y_start
                self.setpoints[i, 2] = -0.5
            elif 49 < i < 100:
                self.setpoints[i, 0] = x_start + 5
                self.setpoints[i, 1] = y_start - (i - 50) * 0.1
                self.setpoints[i, 2] = -0.5
            elif 99 < i < 150:
                self.setpoints[i, 0] = x_start + 5 - (i - 100) * 0.1
                self.setpoints[i, 1] = y_start - 5
                self.setpoints[i, 2] = -0.5
            elif 149 < i < 200:
                self.setpoints[i, 0] = x_start
                self.setpoints[i, 1] = y_start - 5 + (i - 150) * 0.1
                self.setpoints[i, 2] = -0.5

        self.current_setpoint_x = self.setpoints[0, 0]
        self.current_setpoint_y = self.setpoints[0, 1]
        self.current_setpoint_z = self.setpoints[0, 2]
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.look_ahead_distance = 0.3
        self.i = 0  # Laufvariable für setpoint Nummer
        self.init_params()

        self.start_time = self.get_clock().now()

        self.setpoint_pub = self.create_publisher(msg_type=PointStamped,
                                                  topic='setpoint',
                                                  qos_profile=1)

        self.timer = self.create_timer(timer_period_sec=1 / 50,
                                       callback=self.on_timer)

        self.thrust_pub = self.create_publisher(msg_type=ActuatorSetpoint,
                                                topic='thrust_setpoint',
                                                qos_profile=1)
        # self.dvl_yaw_sub = self.create_subscription(
        #     msg_type=DeadReckoningReport,
        #     topic='dead_reckoning_report',
        #     callback=self.on_dvl_yaw,
        #     qos_profile=qos)
        self.position_sub = self.create_subscription(
            msg_type=Odometry,
            topic=
            'state_estimate',  # state_estimate (type: Odometry; weiteres .pose in on_xyz) oder zum Test: ground_truth/pose (type: PoseStamped)
            callback=self.on_xyz,
            qos_profile=1)
        ######################################################################
        self.vision_pose_sub = self.create_subscription(  #simulation
            msg_type=PoseWithCovarianceStamped,
            topic='vision_pose_cov',
            callback=self.on_vision_pose,
            qos_profile=qos)
        ######################################################################
        '''self.position_sub = self.create_subscription(  # für echten ROV (in on_xyz current_X/y/z zweites .pose hinzufügen, an simtime false und bluerov1 denken! vehicle_name:=bluerov01 use_sim_time:=false)
            msg_type=PoseWithCovarianceStamped,
            topic='vision_pose_cov',
            callback=self.on_xyz,
            qos_profile=qos)'''

    def init_params(self):
        self.declare_parameters(
            namespace='',
            parameters=[
                ('gains.p_x', rclpy.Parameter.Type.DOUBLE),
                ('gains.i_x', rclpy.Parameter.Type.DOUBLE),
                ('gains.d_x', rclpy.Parameter.Type.DOUBLE),
                ('limit_integral_x', rclpy.Parameter.Type.DOUBLE),
                ('thrust_offset_x', rclpy.Parameter.Type.DOUBLE),
                ('gains.p_y', rclpy.Parameter.Type.DOUBLE),
                ('gains.i_y', rclpy.Parameter.Type.DOUBLE),
                ('gains.d_y', rclpy.Parameter.Type.DOUBLE),
                ('limit_integral_y', rclpy.Parameter.Type.DOUBLE),
                ('thrust_offset_y', rclpy.Parameter.Type.DOUBLE),
                ('gains.p_z', rclpy.Parameter.Type.DOUBLE),
                ('gains.i_z', rclpy.Parameter.Type.DOUBLE),
                ('gains.d_z', rclpy.Parameter.Type.DOUBLE),
                ('limit_integral_z', rclpy.Parameter.Type.DOUBLE),
                ('thrust_offset_z', rclpy.Parameter.Type.DOUBLE),
            ])
        # get parameters
        param = self.get_parameter('gains.p_x')
        self.get_logger().info(f'{param.name}={param.value}')
        self.p_gain_x = param.value

        param = self.get_parameter('gains.i_x')
        self.get_logger().info(f'{param.name}={param.value}')
        self.i_gain_x = param.value

        param = self.get_parameter('gains.d_x')
        self.get_logger().info(f'{param.name}={param.value}')
        self.d_gain_x = param.value

        param = self.get_parameter('limit_integral_x')
        self.get_logger().info(f'{param.name}={param.value}')
        self.integral_limit_x = param.value

        param = self.get_parameter('thrust_offset_x')
        self.get_logger().info(f'{param.name}={param.value}')
        self.offset_x = param.value

        param = self.get_parameter('gains.p_y')
        self.get_logger().info(f'{param.name}={param.value}')
        self.p_gain_y = param.value

        param = self.get_parameter('gains.i_y')
        self.get_logger().info(f'{param.name}={param.value}')
        self.i_gain_y = param.value

        param = self.get_parameter('gains.d_y')
        self.get_logger().info(f'{param.name}={param.value}')
        self.d_gain_y = param.value

        param = self.get_parameter('limit_integral_y')
        self.get_logger().info(f'{param.name}={param.value}')
        self.integral_limit_y = param.value

        param = self.get_parameter('thrust_offset_y')
        self.get_logger().info(f'{param.name}={param.value}')
        self.offset_y = param.value

        param = self.get_parameter('gains.p_z')
        self.get_logger().info(f'{param.name}={param.value}')
        self.p_gain_z = param.value

        param = self.get_parameter('gains.i_z')
        self.get_logger().info(f'{param.name}={param.value}')
        self.i_gain_z = param.value

        param = self.get_parameter('gains.d_z')
        self.get_logger().info(f'{param.name}={param.value}')
        self.d_gain_z = param.value

        param = self.get_parameter('limit_integral_z')
        self.get_logger().info(f'{param.name}={param.value}')
        self.integral_limit_z = param.value

        param = self.get_parameter('thrust_offset_z')
        self.get_logger().info(f'{param.name}={param.value}')
        self.offset_z = param.value

        self.add_on_set_parameters_callback(self.on_params_changed)

    def on_params_changed(self, params):
        param: rclpy.Parameter
        for param in params:
            self.get_logger().info(f'Try to set [{param.name}] = {param.value}')
            print(vars(param))
            if param.name == 'gains.p_x':
                self.p_gain_x = param.value
            elif param.name == 'gains.i_x':
                self.i_gain_x = param.value
            elif param.name == 'gains.d_x':
                self.d_gain_x = param.value
            elif param.name == 'limit_integral_x':
                self.integral_limit_x = param.value
            elif param.name == 'thrust_offset_x':
                self.offset_x = param.value
            elif param.name == 'gains.p_y':
                self.p_gain_y = param.value
            elif param.name == 'gains.i_y':
                self.i_gain_y = param.value
            elif param.name == 'gains.d_y':
                self.d_gain_y = param.value
            elif param.name == 'limit_integral_y':
                self.integral_limit_y = param.value
            elif param.name == 'thrust_offset_y':
                self.offset_y = param.value
            elif param.name == 'gains.p_z':
                self.p_gain_y = param.value
            elif param.name == 'gains.i_z':
                self.i_gain_y = param.value
            elif param.name == 'gains.d_z':
                self.d_gain_y = param.value
            elif param.name == 'limit_integral_z':
                self.integral_limit_y = param.value
            elif param.name == 'thrust_offset_z':
                self.offset_y = param.value
            else:
                continue
        return SetParametersResult(successful=True, reason='Parameter set')

    def on_timer(self) -> None:
        # change this for other setpoint functions
        now = self.get_clock().now()
        error = math.sqrt((self.current_setpoint_x - self.current_x)**2 +
                          (self.current_setpoint_y - self.current_y)**2 +
                          (self.current_setpoint_z - self.current_z)**2)

        # if error < self.look_ahead_distance:
        #     self.i = self.i + 1
        #     if self.i < 200:
        #         self.current_setpoint_x = self.setpoints[self.i, 0]
        #         self.current_setpoint_y = self.setpoints[self.i, 1]
        #         self.current_setpoint_z = self.setpoints[self.i, 2]

        self.publish_setpoint(setpoint_x=self.current_setpoint_x,
                              setpoint_y=self.current_setpoint_y,
                              setpoint_z=self.current_setpoint_z,
                              now=now)

    def publish_setpoint(self, setpoint_x: float, setpoint_y: float,
                         setpoint_z: float, now: rclpy.time.Time) -> None:
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.point.x = setpoint_x
        msg.point.y = setpoint_y
        msg.point.z = setpoint_z
        self.setpoint_pub.publish(msg)

    def on_dvl_yaw(self, msg: DeadReckoningReport):
        self.yaw = msg.report.rpy.z

    ##################### für Simulation #####################
    def on_vision_pose(self, msg: PoseWithCovarianceStamped):
        # You might want to consider the vehicle's orientation

        # get the vehicle orientation expressed as quaternion
        q = msg.pose.pose.orientation
        # convert quaternion to euler angles
        (roll, pitch, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])

        # TODO
        self.yaw = yaw

    ##########################################################

    def on_xyz(self, xyz_msg: Odometry):
        # We received a new depth message! Now we can get to action!
        current_x = xyz_msg.pose.pose.position.x
        self.current_x = current_x
        current_y = xyz_msg.pose.pose.position.y
        self.current_y = current_y
        current_z = xyz_msg.pose.pose.position.z
        self.current_z = current_z

        thrust_x, thrust_y, thrust_z = self.compute_control_output(
            current_x, current_y, current_z
        )  # thrust x in Absolutkoordinaten (Koordinaten der Umgebung)
        # either set the timestamp to the current time or set it to the
        # stamp of `depth_msg` because the control output corresponds to this
        # point in time. Both choices are meaningful.
        # option 1:
        # now = self.get_clock().now()
        # option 2:
        timestamp = rclpy.time.Time.from_msg(xyz_msg.header.stamp)
        self.publish_thrust(thrust_x=thrust_x,
                            thrust_y=thrust_y,
                            thrust_z=thrust_z,
                            timestamp=timestamp)

    def publish_thrust(self, thrust_x: float, thrust_y: float, thrust_z: float,
                       timestamp: rclpy.time.Time) -> None:
        msg = ActuatorSetpoint()
        # we want to set the robot's x y thrust based on absolute x thrust
        msg.ignore_x = False
        msg.ignore_y = False
        msg.ignore_z = False

        msg.x = thrust_x
        msg.y = thrust_y
        msg.z = thrust_z

        # Let's add a time stamp
        msg.header.stamp = timestamp.to_msg()

        self.thrust_pub.publish(msg)

    # calculating required control output in absolute coordinates first, then transforming into robot coordinates
    def compute_control_output(self, current_x: float, current_y: float,
                               current_z: float):
        global time, previous_time, integral_x, previous_error_x, integral_y, previous_error_y, integral_z, previous_error_z
        # Define parameters
        Kp_x = self.p_gain_x
        Ki_x = self.i_gain_x
        Kd_x = self.d_gain_x
        integral_limit_x = self.integral_limit_x
        offset_x = self.offset_x
        Kp_y = self.p_gain_y
        Ki_y = self.i_gain_y
        Kd_y = self.d_gain_y
        integral_limit_y = self.integral_limit_y
        offset_y = self.offset_y
        Kp_z = self.p_gain_y
        Ki_z = self.i_gain_y
        Kd_z = self.d_gain_y
        integral_limit_z = self.integral_limit_y
        offset_z = self.offset_z

        time = self.get_clock().now()

        # TODO: Apply the PID control
        error_x = self.current_setpoint_x - current_x
        error_y = self.current_setpoint_y - current_y
        error_z = self.current_setpoint_z - current_z

        if previous_time != 0:  # not first loop
            dt = time - previous_time
            proportional_x = error_x
            proportional_y = error_y
            proportional_z = error_z
            integral_x = integral_x + error_x * (dt.nanoseconds / 1e9)
            integral_y = integral_y + error_y * (dt.nanoseconds / 1e9)
            integral_z = integral_z + error_z * (dt.nanoseconds / 1e9)

            # anti-windup
            if integral_x > integral_limit_x:
                integral_x = integral_limit_x
            elif integral_x < -integral_limit_x:
                integral_x = -integral_limit_x

            if integral_y > integral_limit_y:
                integral_y = integral_limit_y
            elif integral_y < -integral_limit_y:
                integral_y = -integral_limit_y

            if integral_z > integral_limit_z:
                integral_z = integral_limit_z
            elif integral_z < -integral_limit_z:
                integral_z = -integral_limit_z

            derivative_x = (error_x - previous_error_x) / (dt.nanoseconds / 1e9)
            derivative_y = (error_y - previous_error_y) / (dt.nanoseconds / 1e9)
            derivative_z = (error_z - previous_error_z) / (dt.nanoseconds / 1e9)
            thrust_x_A = Kp_x * proportional_x + Ki_x * integral_x + Kd_x * derivative_x + offset_x
            thrust_y_A = Kp_y * proportional_y + Ki_y * integral_y + Kd_y * derivative_y + offset_y
            thrust_z_A = Kp_z * proportional_z + Ki_z * integral_z + Kd_z * derivative_z + offset_z
            self.get_logger().info(
                f"Hi! I'm your controller running. "
                f'I received a x of {current_x} m.'
                f'I should be at x: {self.current_setpoint_x} m.'
                f'I calculated thrust {thrust_x_A}.',
                throttle_duration_sec=1)

        else:  # first loop only proportional
            thrust_x_A = Kp_x * error_x + offset_x
            thrust_y_A = Kp_y * error_y + offset_y
            thrust_z_A = Kp_z * error_z + offset_z

        previous_time = time
        previous_error_x = error_x
        previous_error_y = error_y
        previous_error_z = error_z

        # transform thrusts into robot coordinates
        thrust_x_R = thrust_x_A * math.cos(self.yaw) + thrust_y_A * math.sin(
            self.yaw)
        thrust_y_R = -thrust_x_A * math.sin(self.yaw) + thrust_y_A * math.cos(
            self.yaw)
        thrust_z_R = thrust_z_A

        # limit thrusts
        thrust_limit = 0.3
        if thrust_x_R > thrust_limit:
            thrust_x_R = thrust_limit
        elif thrust_x_R < -thrust_limit:
            thrust_x_R = -thrust_limit
        if thrust_y_R > thrust_limit:
            thrust_y_R = thrust_limit
        elif thrust_y_R < -thrust_limit:
            thrust_y_R = -thrust_limit
        if thrust_z_R > thrust_limit:
            thrust_z_R = thrust_limit
        elif thrust_z_R < -thrust_limit:
            thrust_z_R = -thrust_limit

        return thrust_x_R, thrust_y_R, thrust_z_R


def main():
    rclpy.init()
    node = control()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
