#!/usr/bin/env python3

import rospy
import control
from std_msgs.msg import Header
from geometry_msgs.msg import Twist, TwistWithCovariance, PoseWithCovariance
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from projects.msg import Encoders
from tf.transformations import *
from math import sin, cos, pi


class MotorModelController:
    def __init__(self):
        k = 1
        T = 1
        W = control.tf(k, [T, 1])
        self.sys = control.LinearIOSystem(W)
        self.x = [[0, 0]]
        self.t = rospy.Time.now().to_sec()
        self.w_target = 0
        self.enc = 0

    def step(self, w_target):
        t = rospy.Time.now().to_sec()
        a, w, self.x = control.input_output_response(self.sys, [self.t, t], [self.w_target, w_target], self.x[0][1],
                                                     return_x=True)
        dt = t - self.t
        d_enc = int(w[1] * dt / 2 / pi * 4096)

        true_w = d_enc * 2 * pi / dt / 4096

        self.enc += d_enc
        self.t = t
        self.w_target = w_target
        return true_w, dt


class RobotCommander:
    def __init__(self):
        pub1_name = "/enc"
        pub2_name = "/odom"
        sub_name = "/cmd_vel"
        self.pub_enc = rospy.Publisher(pub1_name, Encoders, queue_size=10)
        self.pub_odom = rospy.Publisher(pub2_name, Odometry, queue_size=10)
        self.pub_trajectory_markers = rospy.Publisher("trajectory_markers", Marker, queue_size=10)
        self.sub = rospy.Subscriber(sub_name, Twist, self.handle_cmd_vel)
        self.pose = PoseWithCovariance()
        self.twist = TwistWithCovariance()
        self.left_motor = MotorModelController()
        self.right_motor = MotorModelController()
        self.trajectory_points = []
        self.L = 0.287
        self.r = 0.033
        self.rot = 0

    def handle_cmd_vel(self, msg):
        rospy.loginfo("Received a /cmd_vel message!")
        rospy.loginfo("Linear Component: [%f]" % (msg.linear.x))
        rospy.loginfo("Angular Component: [%f]" % (msg.angular.z))
        self.update_robot_state(msg)

    def update_robot_state(self, msg):
        self.compute_wheel_speeds(msg)
        self.update_motor_state_variables()
        self.update_robot_pose_and_twist()
        self.add_trajectory_point()

    def compute_wheel_speeds(self, msg):
        V = msg.linear.x
        Omega = msg.angular.z
        self.w_l = (V - 0.5 * Omega * self.L) / self.r
        self.w_r = (V + 0.5 * Omega * self.L) / self.r

    def update_motor_state_variables(self):
        true_w_l, self.dt = self.left_motor.step(self.w_l)
        true_w_r, dt = self.right_motor.step(self.w_r)
        self.V = 0.5 * self.r * (true_w_l + true_w_r)
        self.Omega = self.r / self.L * (true_w_r - true_w_l)

    def update_robot_pose_and_twist(self):
        self.rot += self.Omega * self.dt
        x = self.V * cos(self.rot) * self.dt
        self.pose.pose.position.x += x
        y = self.V * sin(self.rot) * self.dt
        self.pose.pose.position.y += y
        q = quaternion_from_euler(0, 0, self.rot)
        self.pose.pose.orientation.x = q[0]
        self.pose.pose.orientation.y = q[1]
        self.pose.pose.orientation.z = q[2]
        self.pose.pose.orientation.w = q[3]
        self.twist.twist.linear.x = self.V * cos(self.rot)
        self.twist.twist.linear.y = self.V * sin(self.rot)
        self.twist.twist.angular.z = self.rot

    def add_trajectory_point(self):
        point = Point()
        point.x = self.pose.pose.position.x
        point.y = self.pose.pose.position.y
        point.z = 0.0
        self.trajectory_points.append(point)

    def publish_enc(self):
        header = Header(stamp=rospy.Time.now(),
                        frame_id="enc")
        msg = Encoders(header=header,
                       left=self.left_motor.enc,
                       right=self.right_motor.enc)

        self.pub_enc.publish(msg)
        rospy.loginfo(msg)

    def publish_odom(self):
        header = Header(stamp=rospy.Time.now(),
                        frame_id="odom")
        msg = Odometry(header=header,
                       child_frame_id="odom",
                       pose=self.pose,
                       twist=self.twist)

        self.pub_odom.publish(msg)
        rospy.loginfo(msg)

    def publish_trajectory_markers(self):
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "trajectory"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.05
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.points = self.trajectory_points
        self.pub_trajectory_markers.publish(marker)

if __name__ == '__main__':
    rospy.init_node("Robot")
    rate = rospy.Rate(10)  # 10hz
    com = RobotCommander()
    while not rospy.is_shutdown():
        com.publish_enc()
        com.publish_odom()
        com.publish_trajectory_markers()
        rate.sleep()

