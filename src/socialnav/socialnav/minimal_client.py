import sys

from my_nav_interface.srv import PathFromModel
import rclpy
from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(PathFromModel, 'path_from_model')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = PathFromModel.Request()

    def send_request(self):
        self.req.init_pose.header.stamp = self.get_clock().now().to_msg()
        self.req.init_pose.pose.position.x = 0.0
        self.req.init_pose.pose.position.y = 0.0
        self.req.init_pose.pose.position.z = 0.0
        self.req.init_pose.pose.orientation.x = 0.0
        self.req.init_pose.pose.orientation.y = 0.0
        self.req.init_pose.pose.orientation.z = 0.0
        self.req.init_pose.pose.orientation.w = 1.0
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main():
    rclpy.init()

    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request()
    print(response)
    minimal_client.get_logger().info(
        'Result of add_two_ints: for' %
        (int(response.path.header.seq)))

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()