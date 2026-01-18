from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class Slam(Node):
    def __init__(self):
        super().__init__('lidar_stream')
        self.lidar_sub = self.create_subscription('/lidar/cloud')
        self.camera_sub = self.create_subscription('/camera1/image')