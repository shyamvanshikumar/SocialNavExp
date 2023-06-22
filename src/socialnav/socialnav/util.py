import numpy as np

class BEVLidar:
    def __init__(self, x_range=(-20, 20),
                 y_range=(-20, 20),
                 resolution=0.05,):
        self.x_range = x_range
        self.y_range = y_range
        self.resolution = resolution
        self.dx = -x_range[0]/resolution
        self.dy = y_range[1]/resolution
        self.img_size = int(1 + (x_range[1] - x_range[0]) / resolution)
        print('created the bev image handler class')
    
    def get_xy_points(self, scan_msg):
        ranges = scan_msg.ranges
        lidar_points = []

        for i, r in enumerate(ranges):
            theta = i*scan_msg.angle_increment
            lidar_points.append((r*np.cos(theta), r*np.sin(theta)))
        
        return lidar_points

    def get_bev_lidar_img(self, lidar_points):
        img = np.zeros((self.img_size, self.img_size))
        for x, y in lidar_points:
            if self.not_in_range_check(x, y): continue
            ix = (self.dx + int(x / self.resolution))
            iy = (self.dy - int(y / self.resolution))
            img[int(round(iy)), int(round(ix))] = 255
        return img.astype(np.uint8)

    def not_in_range_check(self, x, y):
        if x < self.x_range[0] \
                or x > self.x_range[1] \
                or y < self.y_range[0] \
                or y > self.y_range[1]: return True
        return False