import rclpy
from rclpy.node import Node
import rclpy.time
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster, TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped
from bumperbot_msgs.srv import GetTransform
from tf_transformations import quaternion_from_euler, quaternion_multiply, quaternion_inverse

class SimpleTfKinematics(Node):
    def __init__(self):
        super().__init__('simple_tf_kinematics')
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.tf_broadcaster_ = TransformBroadcaster(self)

        self.static_transform_stamped = TransformStamped()
        self.transform_stamped_ = TransformStamped()


        self.last_x = 0.0
        self.x_increment = 0.05
        self.rotations_counter_ = 0
        self.last_orientation = quaternion_from_euler(0, 0, 0)
        self.orientation_increment = quaternion_from_euler(0, 0, 0.05)

        self.last_y = 0.0
        self.y_increment = 0.05

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.static_transform_stamped.header.stamp = self.get_clock().now().to_msg()
        self.static_transform_stamped.header.frame_id = "bumperbot_base"
        self.static_transform_stamped.child_frame_id = "bumperbot_top"
        self.static_transform_stamped.transform.translation.x = 0.0
        self.static_transform_stamped.transform.translation.y = 0.0
        self.static_transform_stamped.transform.translation.z = 0.3
        self.static_transform_stamped.transform.rotation.x = 0.0
        self.static_transform_stamped.transform.rotation.y = 0.0
        self.static_transform_stamped.transform.rotation.z = 0.0
        self.static_transform_stamped.transform.rotation.w = 1.0

       
        
        self.tf_static_broadcaster.sendTransform(self.static_transform_stamped)

        self.get_logger().info(f'Publishing static transform between {self.static_transform_stamped.header.frame_id} and {self.static_transform_stamped.child_frame_id }')
        self.timer = self.create_timer(0.1, self.timerCallBack)
        self.get_transform_srv = self.create_service(GetTransform, "get_transform", self.getTransformCallback)


    def timerCallBack(self):
        self.transform_stamped_.header.stamp = self.get_clock().now().to_msg()
        self.transform_stamped_.header.frame_id = "odom"
        self.transform_stamped_.child_frame_id = "bumperbot_base"
        self.transform_stamped_.transform.translation.x = self.last_x + self.x_increment
        self.transform_stamped_.transform.translation.y = self.last_y + self.y_increment
        self.transform_stamped_.transform.translation.z = 0.0

        q = quaternion_multiply(self.last_orientation, self.orientation_increment)
        self.transform_stamped_.transform.rotation.x = q[0]
        self.transform_stamped_.transform.rotation.y = q[1]
        self.transform_stamped_.transform.rotation.z = q[2]
        self.transform_stamped_.transform.rotation.w = q[3]
        self.tf_broadcaster_.sendTransform(self.transform_stamped_)

        self.last_x = self.transform_stamped_.transform.translation.x
        self.last_y = self.transform_stamped_.transform.translation.y
        self.rotations_counter_+= 1
        self.last_orientation = q

        if self.rotations_counter_>= 100:
            self.orientation_increment = quaternion_inverse(self.orientation_increment)
            self.rotations_counter_ = 0


    def getTransformCallback(self, req, res):
        self.get_logger().info("Requested transform between %s and %s" % (req.frame_id, req.child_frame_id))
        requested_tf = TransformStamped()
        try:
            requested_tf  = self.tf_buffer.lookup_transform(req.frame_id, req.child_frame_id, rclpy.time.Time())
        except TransformException as e:
            self.get_logger().error("An error has occured while transforming %s and %s" % (req.frame_id, req.child_frame_id))
            res.success = False
            return res
        
        res.transform = requested_tf
        res.success = True
        return res

def main():
    rclpy.init()
    simple_tf_kinematics = SimpleTfKinematics()
    rclpy.spin(simple_tf_kinematics)
    simple_tf_kinematics.destroy_node
    rclpy.shutdown


if __name__ == "__main__":
    main()