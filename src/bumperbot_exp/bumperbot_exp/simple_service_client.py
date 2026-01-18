import rclpy
from rclpy.node import Node
from bumperbot_msgs.srv import AddTwoInts
import sys


class SimpleServiceClient(Node):
    def __init__(self, a, b):
        super().__init__("simple_service_client")

        self.client_ = self.create_client(AddTwoInts, 'add_two_ints')

        while not self.client_.wait_for_service(timeout_sec=1):
            self.get_logger().info("Service not available; waiting...")

        self.req = AddTwoInts.Request()
        self.req.a = a
        self.req.b = b
        self.future = self.client_.call_async(self.req)
        self.future.add_done_callback(self.responseCallback)

    def responseCallback(self, future):
        self.get_logger().info("service response %d" % future.result().sum)


def main():
    rclpy.init()

    if len(sys.argv) !=3:
        print("Wrong number of arguments. Usage: simple_service_client A B")
        return -1
    
    simple_service_client = SimpleServiceClient(int(sys.argv[1]), int(sys.argv[2]))
    rclpy.spin(simple_service_client)
    simple_service_client.destroy_node()

    rclpy.shutdown

if __name__ == '__main__':
    main()
