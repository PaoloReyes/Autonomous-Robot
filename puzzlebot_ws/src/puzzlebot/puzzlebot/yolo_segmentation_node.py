import os
import cv2

import rclpy
from rclpy import qos
from rclpy.node import Node

from sensor_msgs.msg import Image

from std_msgs.msg import Int32MultiArray, String

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
        self.boxes_pub = self.create_publisher(Image, '/boxes', qos.qos_profile_sensor_data)
        self.direction_pub = self.create_publisher(String, '/direction', 10)
        self.behaviour_pub = self.create_publisher(String, '/behaviour', 10)
        self.light_pub = self.create_publisher(String, '/light', 10)
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

        self.focal_lenght = 176.16
        self.traffic_distance = 6.0

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
            groups = [[], [], []]
            for c in result:
                label = c.names[c.boxes.cls.tolist().pop()]
                if label == 'street':
                    contour = c.masks.xy.pop().astype(np.int32).reshape(-1, 1, 2)
                    _ = cv2.drawContours(b_mask, [contour], -1, (255, 255, 255), cv2.FILLED)
                elif label == 'forward' or label == 'left' or label == 'right':
                    box = c.boxes.xyxy.tolist()[0]
                    label = c.names[c.boxes.cls.tolist().pop()]
                    confidence = c.boxes.conf.tolist().pop()
                    groups[0].append((box, label, confidence))
                elif label == 'green' or label == 'red' or label == 'yellow':
                    box = c.boxes.xyxy.tolist()[0]
                    label = c.names[c.boxes.cls.tolist().pop()]
                    confidence = c.boxes.conf.tolist().pop()
                    groups[1].append((box, label, confidence))
                elif label == 'stop' or label == 'workers' or label == 'give_way':
                    box = c.boxes.xyxy.tolist()[0]
                    label = c.names[c.boxes.cls.tolist().pop()]
                    confidence = c.boxes.conf.tolist().pop()
                    groups[2].append((box, label, confidence))
            
            unique_groups = [(), (), ()]
            for i, group in enumerate(groups):
                if len(group) > 0:
                    unique_groups[i] = sorted(group, key=lambda x: x[2], reverse=True)[0]
            
                
            boxes_img = raw.copy()
            for unique_group in unique_groups:
                try:
                    r, g, b = np.random.randint(0, 255), np.random.randint(0, 255), np.random.randint(0, 255)
                    cv2.rectangle(boxes_img, (int(unique_group[0][0]), int(unique_group[0][1])), (int(unique_group[0][2]), int(unique_group[0][3])), (r, g, b), 2)
                    cv2.putText(boxes_img, f'{unique_group[1]} {unique_group[2]:.2f}', (int(unique_group[0][0]), int(unique_group[0][1])), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (r, g, b), 2)
                except:
                    pass

            # Find the distance to the camera of each of the unique boxes
            for i, unique_group in enumerate(unique_groups):
                if len(unique_group) > 0:
                    box = unique_group[0]
                    x, y = (box[0] + box[2])//2, (box[1] + box[3])//2
                    z = math_utils.distance_to_camera(self.focal_lenght, self.traffic_distance, box[2] - box[0]) #in centimeters
                    if z < 20:
                        if i == 0:
                            msg = String()
                            msg.data = unique_group[1]
                            self.direction_pub.publish(msg)
                        elif i == 1:
                            msg = String()
                            msg.data = unique_group[1]
                            self.light_pub.publish(msg)
                        elif i == 2:
                            msg = String()
                            msg.data = unique_group[1]
                            self.behaviour_pub.publish(msg)



            blurred_mask = cv2.GaussianBlur(b_mask, (15, 15), 0)

            contours, _ = cv2.findContours(blurred_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
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

                msg = Int32MultiArray()
                msg.data = [x, y]
                self.CoM_pub.publish(msg)
        
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(inference, encoding='bgr8'))
            self.boxes_pub.publish(self.bridge.cv2_to_imgmsg(boxes_img, encoding='bgr8'))

def main(args=None):
    rclpy.init(args=args)
    yolo_node = YOLONode()
    rclpy.spin(yolo_node)
    yolo_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()