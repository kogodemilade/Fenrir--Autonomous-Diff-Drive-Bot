import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose


class TurtleSimRotationMatrix(Node):
    def __init__(self):
        super().__init__("turtlesim_rotation_matrix")

        self.turtle_1_pose_sub = self.create_subscription(Pose, "/turtle1/pose", self.turtle1Callback, 10)
        self.turtle_2_pose_sub = self.create_subscription(Pose, "/turtle2/pose", self.turtle2Callback, 10)
        self.t1_pose = Pose()
        self.t2_pose = Pose()

    def turtle1Callback(self, msg):
        self.t1_pose = msg

    def turtle1Callback(self, msg):
        self.t2_pose = msg

        t = self.t2_pose.theta - self.t1_pose.theta
        xw = self.t2_pose.x
        