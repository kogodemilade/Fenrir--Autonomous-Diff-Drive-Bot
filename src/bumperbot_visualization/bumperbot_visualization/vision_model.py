import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO


class VisionModel(Node):
    def __init__(self):
        super().__init__('vision_model')
        self.sub = self.create_subscription(Image, '/camera1/preprocessed_out', self.cb, 10)
        self.pub = self.create_publisher(Image, '/camera1/vision', 10)
        # self.declare_parameter('use_sim_time', True)
        self.model = YOLO("yolo11n.pt")
        # self.model = YOLO('yolov8n-seg.pt')
        self.window_name = "live-vision"
        cv2.namedWindow(self.window_name, cv2.WINDOW_AUTOSIZE)
        self.br = CvBridge()


    def cb(self, msg):
        cv_img = self.br.imgmsg_to_cv2(msg, 'bgr8')
        # cv_img = np.clip(cv_img * 255.0, 0, 255).astype(np.uint8)
        # cv_img = cv2.cvtColor(cv_img, cv2.COLOR_RGB2BGR)
        result = self.model(cv_img)[0]
        # boxes = result.boxes 
        # masks = result.masks  
        # keypoints = result.keypoints 
        # probs = result.probs  
        # obb = result.obb  # Oriented boxes object for OBB outputs
        # result.show()  # display to screen
        annotated = result.plot()         # BGR numpy array
        cv2.imshow(self.window_name, annotated)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rclpy.shutdown()
        new_msg = self.br.cv2_to_imgmsg(annotated, encoding='bgr8')
        self.pub.publish(new_msg)


    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()

def main():
    rclpy.init()
    node = VisionModel()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
