import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from message_filters import ApproximateTimeSynchronizer, Subscriber

class SyncTimestampLogger(Node):
    def __init__(self):
        super().__init__('sync_timestamp_logger')
        
        # Subscribers for rgb image, depth image, and camera info
        self.rgb_sub = Subscriber(self, Image, '/camera1/image/image')
        self.depth_sub = Subscriber(self, Image, '/camera1/image/depth_image')
        self.cam_info_sub = Subscriber(self, CameraInfo, '/camera1/image/camera_info')
        
        # ApproximateTimeSynchronizer for syncing the three topics
        self.ts = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub, self.cam_info_sub],
            queue_size=10,
            slop=0.1,  # max time difference (in seconds) allowed for sync
            allow_headerless=False
        )
        self.ts.registerCallback(self.callback)
    
    def callback(self, rgb_msg, depth_msg, cam_info_msg):
        rgb_time = rgb_msg.header.stamp.sec + rgb_msg.header.stamp.nanosec * 1e-9
        depth_time = depth_msg.header.stamp.sec + depth_msg.header.stamp.nanosec * 1e-9
        cam_info_time = cam_info_msg.header.stamp.sec + cam_info_msg.header.stamp.nanosec * 1e-9
        
        diff_rgb_depth = abs(rgb_time - depth_time)
        diff_rgb_caminfo = abs(rgb_time - cam_info_time)
        diff_depth_caminfo = abs(depth_time - cam_info_time)
        
        self.get_logger().info(
            f"RGB: {rgb_time:.6f}, Depth: {depth_time:.6f}, CamInfo: {cam_info_time:.6f}\n"
            f"Differences (s): RGB-Depth={diff_rgb_depth:.6f}, "
            f"RGB-CamInfo={diff_rgb_caminfo:.6f}, Depth-CamInfo={diff_depth_caminfo:.6f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = SyncTimestampLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
