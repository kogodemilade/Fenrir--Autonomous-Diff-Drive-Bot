import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose

class SimpleTurtlesimKinematics(Node):
    def __init__(self):
        super().__init__("simple_turtlesim_kinematics")

        self.turtle_1_pose_sub = self.create_subscription(Pose, "/turtle1/pose", self.Turtle1Callback,10)
        self.turtle_2_pose_sub = self.create_subscription(Pose, "/turtle2/pose", self.Turtle2Callback, 10)
        self.last_t1_pose = Pose()
        self.last_t2_pose = Pose()

    def Turtle1Callback(self, msg):
        self.last_t1_pose = msg


    def Turtle2Callback(self, msg):
        self.last_t2_pose = msg

        Tx = self.last_t2_pose.x - self.last_t1_pose.x
        Ty= self.last_t2_pose.y - self.last_t1_pose.y

        self.get_logger().info(f"""\n
                    Translation Vector between turtle1 -> turtle2 \n
                    Tx: {Tx} \n
                    Ty: {Ty} \n""")
        


def main():
    rclpy.init()
    kinematic = SimpleTurtlesimKinematics()
    rclpy.spin(kinematic)
    kinematic.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

    