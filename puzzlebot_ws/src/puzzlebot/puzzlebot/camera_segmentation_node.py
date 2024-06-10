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

from cv_bridge import CvBridge

from .submodules import camera_utils

from ament_index_python import get_package_share_directory

from copy import deepcopy

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_segmentation_node')
        self.bridge = CvBridge()
        
        # Publisher
        self.pub = self.create_publisher(String, 'debug', 10)
        self.coord_pub = self.create_publisher(Int32MultiArray, 'street_coords', 10)

        # Open the camera feed
        self.source = cv2.VideoCapture(self.gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)

        import torch
        from ultralytics import YOLO

        # Timers
        timer_period = 0.08 
        self.timer = self.create_timer(timer_period, self.timer_callback)
    
        # Import YOLO model
        path = get_package_share_directory('puzzlebot')
        path = path.split('install')[0]
        path = os.path.join(path, 'src', 'puzzlebot','package_data','best.pt')

        self.model = YOLO(path)
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        print('\nUsing device:', self.device)
        self.model.model.eval()
        self.model.to(self.device)

    def timer_callback(self):
        if self.source.isOpened():
            _, img = self.source.read()
            img = cv2.flip(img, 0)
            img = cv2.flip(img, 1)
            raw = camera_utils.undistort(img, (320, 240))
    
            import torch
    
            with torch.no_grad():
                result = self.model(raw)[0]
                inference = result.plot()

            b_mask = np.zeros(img.shape[:2], np.uint8)
            boxes = []
            for c in result:
                label = c.names[c.boxes.cls.tolist().pop()]
                if label == 'street':
                    contour = c.masks.xy.pop().astype(np.int32).reshape(-1, 1, 2)
                    _ = cv2.drawContours(b_mask, [contour], -1, (255, 255, 255), cv2.FILLED)
                else:
                    boxes.append((c.xyxy[0].cpu().numpy().astype(np.int32), c.conf[0].cpu().numpy().astype(np.float32)))
            
            indexes_to_pop = []
            for i, box_1 in enumerate(boxes):
                for j , box_2 in enumerate(boxes):
                    if i != j:
                        x1, y1, x2, y2 = box_1[0]
                        x3, y3, x4, y4 = box_2[0]
                        if (x1 > x3 and x1 < x4 and y1 > y3 and y1 < y4) or (x3 > x1 and x3 < x2 and y3 > y1 and y3 < y2):
                            x5, y5, x6, y6 = (max(x1, x3), max(y1, y3), min(x2, x4), min(y2, y4))
                            intersection = (x6 - x5) * (y6 - y5)
                            union = (x2 - x1) * (y2 - y1) + (x4 - x3) * (y4 - y3) - intersection
                            IoU = intersection / union
                            if IoU > 0.5:
                                if box_1[1] > box_2[1]:
                                    indexes_to_pop.append(j)
                                else:
                                    indexes_to_pop.append(i)
            set_indexes = set(indexes_to_pop)
            for index in set_indexes:
                boxes.pop(index)
            for i, box in enumerate(boxes):
                boxes[i] = box[0]

            boxes_img = raw.compy().astype(np.int32)
            for box in boxes:
                x1, y1, x2, y2 = box
                cv2.rectangle(boxes_img, (x1, y1), (x2, y2), (np.random.randint(0, 255), np.random.randint(0, 255), np.random.randint(0, 255)), 2)

            merged_mask = cv2.cvtColor(b_mask, cv2.COLOR_GRAY2BGR)
            blurred_mask = cv2.GaussianBlur(merged_mask, (15, 15), 0)
            img_masked = cv2.bitwise_and(blurred_mask, raw)
            edges = cv2.Canny(blurred_mask, 100, 200)

            contours, _ = cv2.findContours(b_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if len(contours) > 0:
                max_c = max(contours, key=cv2.contourArea)
                M = cv2.moments(max_c)
                if M['m00'] != 0:
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])
                    print(f'distance in y: {cy}')
                    cv2.circle(edges, (cx, cy), 5, (255, 255, 255), -1)

                # Mapping the x and y coordinates of the center of the street to the center of the image
                x = raw.shape[1]//2 - cx
                y = 0
                if cy > 190 and cy < 240:
                    y = self.map(cy, 190, 240, 50, 0)
                if cy < 190:
                    y = 50

                msg = Int32MultiArray()
                msg.data = [x, y]
                self.coord_pub.publish(msg)


            cv2.imshow('Original Image', raw)
            cv2.imshow('boxes', boxes_img)
            cv2.imshow('street', img_masked)
            cv2.imshow('edges', edges)
            cv2.imshow('YOLOv8 Inference', inference)
            cv2.waitKey(1)

            msg = String()
            msg.data = result.verbose()

            self.pub.publish(msg)
            #msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
            #self.pub.publish(msg)

        else:
            print('Unable to open camera')
    
    def gstreamer_pipeline(self, sensor_id=0, capture_width=320, capture_height=240, display_width=320, display_height=240,framerate=5, flip_method=0):
        return (
            "nvarguscamerasrc sensor-id=%d ! "
            "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
            "nvvidconv flip-method=%d ! "
            "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
            "videoconvert ! "
            "video/x-raw, format=(string)BGR ! appsink"
            % (sensor_id, capture_width, capture_height, framerate, flip_method, display_width, display_height)
        )

    def map(self, x: int, in_min: int, in_max: int, out_min: int, out_max: int) -> int:
        return int((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)
          
def main(args=None):
    rclpy.init(args=args)
    camera_node = CameraNode()
    rclpy.spin(camera_node)
    camera_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()