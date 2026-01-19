import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, LaserScan, PointCloud2
from cv_bridge import CvBridge
import cv2

class CameraStream(Node):
    def __init__(self):
        super().__init__('camera_stream')
        self.sub = self.create_subscription(Image, '/camera1/image/image', self.cb, 100)
        self.depth_sub = self.create_subscription(Image, '/camera1/image/depth_image', self.dcb, 100)
        self.info_sub = self.create_subscription(CameraInfo, '/camera1/image/camera_info', self.icb, 100)
        self.lidar_sub = self.create_subscription(LaserScan, '/lidar/cloud', self.lidarcb, 100)
        self.lidar_3D_sub = self.create_subscription(PointCloud2, '/lidar/cloud/points', self.lidar3Dcb, 100)

        self.pub = self.create_publisher(Image, '/camera1/preprocessed_out', 100)
        self.repub = self.create_publisher(Image, '/camera1/image', 100)
        self.depth_repub = self.create_publisher(Image, '/camera1/depth_image', 100)
        self.info_repub = self.create_publisher(CameraInfo, '/camera1/camera_info', 100)
        self.lidar_repub = self.create_publisher(LaserScan, '/lidar/scan', 100)
        self.lidar_3D_repub = self.create_publisher(PointCloud2, '/lidar/scan/points', 100)
        self.br = CvBridge()

    def cb(self, msg):
        self.fixImageFrame(msg)
        cv_img = self.br.imgmsg_to_cv2(msg, 'bgr8')
        cv_img = self.preprocess(cv_img)
        new_msg = self.br.cv2_to_imgmsg(cv_img, 'bgr8')
        self.pub.publish(new_msg)

    def lidarcb(self, msg):
        msg.header.frame_id = 'lidar_link'
        self.lidar_repub.publish(msg)

    def lidar3Dcb(self, msg):
        msg.header.frame_id = 'lidar_link'
        self.lidar_3D_repub.publish(msg)

    def dcb(self, msg):
        msg.header.frame_id = 'camera_link'
        self.depth_repub.publish(msg)

    def icb(self, msg):
        msg.header.frame_id = 'camera_link'
        self.info_repub.publish(msg)

    def fixImageFrame(self, msg):
        msg.header.frame_id = 'camera_link'
        self.repub.publish(msg)


    def preprocess(self, img):
        cv_img = cv2.resize(img, (320, 320))
        return cv_img
    

def main():
    rclpy.init()
    node = CameraStream()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


