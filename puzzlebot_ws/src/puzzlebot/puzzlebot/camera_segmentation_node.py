"""
This node is responsible for capturing the camera feed and performing segmentation on the image.
The segmentation is done using the YOLOv8 model. The model is imported from the ultralytics library.
The model is used to detect the street in the image. The detected street is then used to calculate the distance
between the center of the street and the center of the image. This distance is then used to calculate the x and y
coordinates of the center of the street with respect to the center of the image. The x and y coordinates are then
used to calculate the distance between the robot and the street. The distance is then used to control the robot.
The node also publishes the image with the detected street and the distance between the robot and the street.

Authors: RikuNav & PaoloReyes
"""
import cv2
import numpy as np
import os

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Int32MultiArray

from sensor_msgs.msg import Image

from cv_bridge import CvBridge

from .submodules import camera_utils

from ament_index_python import get_package_share_directory

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_segmentation_node')
        self.bridge = CvBridge()
        
        # Publisher
        self.pub = self.create_publisher(Image, 'raw_image', 10)

        # Open the camera feed
        self.source = cv2.VideoCapture(self.gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)

        # Timers
        timer_period = 0.0666
        self.timer = self.create_timer(timer_period, self.timer_callback)
    
    def timer_callback(self):
        if self.source.isOpened():
            _, img = self.source.read()
            img = cv2.flip(img, 0)
            img = cv2.flip(img, 1)
            raw = camera_utils.undistort(img, (320, 240))
    
            msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
            self.pub.publish(msg)

        else:
            print('Unable to open camera')
    
    def gstreamer_pipeline(self, sensor_id=0, capture_width=320, capture_height=240, display_width=320, display_height=240, framerate=15, flip_method=0):
        return (
            "nvarguscamerasrc sensor-id=%d ! "
            "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
            "nvvidconv flip-method=%d ! "
            "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
            "videoconvert ! "
            "video/x-raw, format=(string)BGR ! appsink"
            % (sensor_id, capture_width, capture_height, framerate, flip_method, display_width, display_height)
        )
          
def main(args=None):
    rclpy.init(args=args)
    camera_node = CameraNode()
    rclpy.spin(camera_node)
    camera_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()