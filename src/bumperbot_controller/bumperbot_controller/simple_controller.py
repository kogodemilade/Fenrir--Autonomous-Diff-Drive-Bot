#!/usr/bin/env python3

import rclpy, numpy as np
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TwistStamped, TransformStamped
from sensor_msgs.msg import JointState
from rclpy.time import Time
from rclpy.constants import S_TO_NS
import math
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster


class SimpleController(Node):
    def __init__(self):
        super().__init__("simple_controller")

        self.declare_parameter("wheel_radius", 0.033)
        self.declare_parameter("wheel_seperation", 0.17)

        self.wheel_radius_ = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.wheel_sep_ = self.get_parameter("wheel_seperation").get_parameter_value().double_value

        self.get_logger().info(f"Using wheel radius: {self.wheel_radius_}.")
        self.get_logger().info(f"Using wheel seperation: {self.wheel_sep_}.")

        self.left_wheel_prev_pos = 0.0
        self.right_wheel_prev_pod = 0.0
        self.prev_time = self.get_clock().now()

        self.x_ = 0.0
        self.y_ = 0.0
        self.theta_ = 0.0

        self.wheel_cmd_pub_ = self.create_publisher(Float64MultiArray, "simple_velocity_controller/commands", 10)
        self.get_logger().info('Created Wheel Command Publisher node')
        self.vel_sub_ = self.create_subscription(TwistStamped, "bumperbot_controller/cmd_vel", self.VelCallback, 10)
        self.get_logger().info('Created Velocity Command Subscriber node')
        self.joint_sub_ = self.create_subscription(JointState, "joint_states", self.jointCallback, 10)
        self.get_logger().info('Created joint states Subscriber node')

        self.odom_pub_ = self.create_publisher(Odometry, "bumperbot_controller/odom", 5)
        self.get_logger().info('Created odom Publisher node')
        self.speed_conversion_ = np.array([[self.wheel_radius_/2, self.wheel_radius_/2], 
                                           [self.wheel_radius_/self.wheel_sep_, -self.wheel_radius_/self.wheel_sep_]])
        
        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = "odom"
        self.odom_msg.child_frame_id = "base_footprint"
        self.odom_msg.pose.pose.orientation.x = 0.0
        self.odom_msg.pose.pose.orientation.y = 0.0
        self.odom_msg.pose.pose.orientation.z = 0.0
        self.odom_msg.pose.pose.orientation.w = 1.0

        self.br_ = TransformBroadcaster(self,)
        self.transform = TransformStamped()
        self.transform.header.frame_id = "odom"
        self.transform.child_frame_id = "base_footprint"
        
        self.get_logger().info(f"The matrix is: {self.speed_conversion_}")

    def VelCallback(self, msg):
        robot_speed = np.array([[msg.twist.linear.x], 
                                [msg.twist.angular.z]])
        
        wheel_speed = np.matmul(np.linalg.inv(self.speed_conversion_), robot_speed)

        wheel_speed_msg =  Float64MultiArray()
        wheel_speed_msg.data = [wheel_speed[1, 0], wheel_speed[0, 0]]

        self.wheel_cmd_pub_.publish(wheel_speed_msg)


    def jointCallback(self, msg):
            dp_left = msg.position[1] - self.left_wheel_prev_pos
            dp_right = msg.position[0] - self.right_wheel_prev_pod
            current_time = Time.from_msg(msg.header.stamp)
            dt = current_time - self.prev_time

            if dt.nanoseconds > 1000:
                # your update logic
                self.prev_time = current_time
            else:
                # self.get_logger().warn("Same timestamp — skipping")
                return

            self.left_wheel_prev_pos = msg.position[1]
            self.right_wheel_prev_pod = msg.position[0]
            # self.prev_time = Time.from_msg(msg.header.stamp)

            phi_left = dp_left/(dt.nanoseconds / S_TO_NS)
            phi_right = dp_right/(dt.nanoseconds / S_TO_NS)
            
            self.linear_vel = ((self.wheel_radius_*phi_right) + (self.wheel_radius_*phi_left))/2
            self.angular_vel = ((self.wheel_radius_*phi_right) - (self.wheel_radius_*phi_left))/self.wheel_sep_


            d_s = (self.wheel_radius_ * dp_right + self.wheel_radius_ * dp_left)/2
            d_theta = (self.wheel_radius_ * dp_right - self.wheel_radius_ * dp_left)/self.wheel_sep_
            self.theta_ += d_theta
            self.x_ += d_s*math.cos(self.theta_)
            self.y_ += d_s*math.sin(self.theta_)

            q = quaternion_from_euler(0, 0, self.theta_)
            self.odom_msg.pose.pose.orientation.x = q[0]
            self.odom_msg.pose.pose.orientation.y = q[1]
            self.odom_msg.pose.pose.orientation.z = q[2]
            self.odom_msg.pose.pose.orientation.w = q[3]
            self.odom_msg.header.stamp = self.get_clock().now().to_msg()
            self.odom_msg.pose.pose.position.x = self.x_
            self.odom_msg.pose.pose.position.y = self.y_
            self.odom_msg.twist.twist.linear.x = self.linear_vel
            self.odom_msg.twist.twist.angular.z = self.angular_vel

            self.transform.transform.translation.x = self.x_
            self.transform.transform.translation.y = self.y_
            self.transform.transform.rotation.x =  q[0]
            self.transform.transform.rotation.y =  q[1]
            self.transform.transform.rotation.z =  q[2]
            self.transform.transform.rotation.w =  q[3]
            self.transform.header.stamp = self.get_clock().now().to_msg()
            
            
            self.odom_pub_.publish(self.odom_msg)
            self.br_.sendTransform(self.transform)
            # self.get_logger().info("linear: %f, angular: %f, x: %f, y: %f, orientation: %f" % (self.linear_vel, self.angular_vel, self.x_, self.y_, self.theta_))

def main():
    rclpy.init()
    controller = SimpleController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()