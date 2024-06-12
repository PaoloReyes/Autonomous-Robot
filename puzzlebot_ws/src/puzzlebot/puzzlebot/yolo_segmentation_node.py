import os
import cv2

import rclpy
from rclpy import qos
from rclpy.node import Node

from sensor_msgs.msg import Image

from std_msgs.msg import Int32MultiArray

from cv_bridge import CvBridge

import torch

from ultralytics import YOLO

from ament_index_python.packages import get_package_share_directory

from .submodules import math_utils, camera_utils

import numpy as np

class YOLONode(Node):
    def __init__(self):
        super().__init__('yolo_segmentation_node')
        self.bridge = CvBridge() 

        self.image_sub = self.create_subscription(Image, '/video_source/raw', self.image_callback, qos.qos_profile_sensor_data)
        self.image_pub = self.create_publisher(Image, '/inference', qos.qos_profile_sensor_data)

        self.CoM_pub = self.create_publisher(Int32MultiArray, 'CoM', qos.qos_profile_sensor_data)

        inference_frequency = 30
        self.timer = self.create_timer(1/inference_frequency, self.timer_callback)

        # Import YOLO model
        path = get_package_share_directory('puzzlebot')
        path = path.split('install')[0]
        path = os.path.join(path, 'src', 'puzzlebot','package_data','best.pt')

        self.model = YOLO(path)
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.get_logger().info(f'Using device: {self.device}')
        self.model.model.eval()
        self.model.to(self.device)

        self.image = None

    def image_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    
    def timer_callback(self):
        if self.image is not None:
            raw = self.image.copy()
            raw = camera_utils.undistort(raw, (320, 240)) 

            with torch.no_grad():
                result = self.model(raw, verbose=False)[0]
                inference = result.plot()

            b_mask = np.zeros(raw.shape[:2], np.uint8)
            for c in result:
                label = c.names[c.boxes.cls.tolist().pop()]
                if label == 'street':
                    contour = c.masks.xy.pop().astype(np.int32).reshape(-1, 1, 2)
                    _ = cv2.drawContours(b_mask, [contour], -1, (255, 255, 255), cv2.FILLED)

            blurred_mask = cv2.GaussianBlur(b_mask, (15, 15), 0)

            contours, _ = cv2.findContours(blurred_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            msg = Int32MultiArray()
            if len(contours) > 0:
                max_c = max(contours, key=cv2.contourArea)
                M = cv2.moments(max_c)
                if M['m00'] != 0:
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])
                    cv2.circle(inference, (cx, cy), 5, (255, 0, 0), -1)

                # Mapping the x and y coordinates of the center of the street to the center of the image
                x = raw.shape[1]//2 - cx
                if cy > 163 and cy < 180:
                    y = math_utils.map(cy, 163, 180, 50, 0)
                elif cy > 180:
                    y = 0
                else:
                    y = 50

                msg.data = [x, y]
            else:
                msg.data = [0, 0]
            
            self.CoM_pub.publish(msg)
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(inference, encoding='bgr8'))

def main(args=None):
    rclpy.init(args=args)
    yolo_node = YOLONode()
    rclpy.spin(yolo_node)
    yolo_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()