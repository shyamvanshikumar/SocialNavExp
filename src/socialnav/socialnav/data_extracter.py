import rclpy
import yaml
import os
import cv2
import numpy as np
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import message_filters
from .util import BEVLidar
from sensor_msgs.msg import LaserScan, Image


class DataExtracter(Node):

    def __init__(self, config_path, save_dir):
        super().__init__('data_extracter')

        lidar = message_filters.Subscriber(self, LaserScan, '/scan')
        rgb_img = message_filters.Subscriber(self, Image, '/color/preview/image')

        ts = message_filters.ApproximateTimeSynchronizer(
            [lidar, rgb_img], 100, 1000000000, allow_headerless=True)
        ts.registerCallback(self.callback)

        self.config = yaml.safe_load(open(config_path, 'r'))
        print('Config file loaded!')
        print(self.config)

        self.save_dir = save_dir

        self.bevlidar_handler = BEVLidar(x_range=(-self.config['LIDAR_BACK_RANGE'], self.config['LIDAR_FRONT_RANGE']),
                                         y_range=(-self.config['LIDAR_SIDE_RANGE'], self.config['LIDAR_SIDE_RANGE']),
                                         resolution=self.config['RESOLUTION'])
        
        print("init_complete")
    
    def callback(self, lidar_scan, rgb_img):
        print("hh")
        lidar_points = self.bevlidar_handler.get_xy_points(lidar_scan)
        lidar_bev = self.bevlidar_handler.get_bev_lidar_img(lidar_points)

        h, w = rgb_img.height, rgb_img.width
        rgb_img = np.array(rgb_img.data).reshape((h,w,3))

        lidar_path = os.path.join(self.save_dir, 'lidar.png')
        rgb_path = os.path.join(self.save_dir, 'rgb_img.png')

        if not cv2.imwrite(lidar_path, lidar_bev):
            raise Exception('Could not write lidar image')
        
        if not cv2.imwrite(rgb_path, rgb_img):
            raise Exception('Could not write rgb image')
        
        print("Collected something!!!")


def main(args=None):
    rclpy.init(args=args)

    data_extracter = DataExtracter(config_path='./src/socialnav/socialnav/config.yaml', save_dir='./src/socialnav/data')

    rclpy.spin(data_extracter)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    data_extracter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
