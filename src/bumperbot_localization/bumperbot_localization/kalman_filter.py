#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

class KalmanFilter(Node):
    def __init__(self):
        super().__init__('Kalman_filter')

        self.odom_sub_ = self.create_subscription(Odometry, "bumperbot_controller/odom_noisy", self.odomCallback, 10)
        self.imu_sub_ = self.create_subscription(Imu, "imu", self.imuCallback, 10)

        self.odom_pub = self.create_publisher(Odometry, "bumperbot_controller/odom_kalman", 10)

        self.mean = 0.0
        self.variance = 1000.0

        self.imu_angular_z = 0.0
        self.first = True
        self.last_angular_z = 0.0

        self.motion = 0.0
        self.kalman_odom = Odometry()

        self.motion_variance = 4.0
        self.measurement_variance = 0.5


    def measurementUpdate(self):
        self.mean = (self.measurement_variance * self.mean  + self.variance *self.imu_angular_z)/(self.variance + self.measurement_variance)
        self.variance = (self.variance * self.measurement_variance)/(self.variance + self.measurement_variance)


    def statePrediction(self):
        self.mean = self.mean + self.motion
        self.variance = self.variance + self.motion_variance


    def imuCallback(self, imu: Imu):
        self.imu_angular_z = imu.angular_velocity.z

    
    def odomCallback(self, odom: Odometry):
        self.kalman_odom = odom

        if self.first:
            self.mean = odom.twist.twist.angular.z
            self.last_angular_z = odom.twist.twist.angular.z
            self.first = False
            return
        
        self.motion = odom.twist.twist.angular.z - self.last_angular_z
        


        self.statePrediction()
        self.measurementUpdate()

        self.kalman_odom.twist.twist.angular.z = self.mean
        self.odom_pub.publish(self.kalman_odom)


def main():
    rclpy.init()
    kalman_filter = KalmanFilter()
    rclpy.spin(kalman_filter)
    kalman_filter.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()


    