import cv2
import numpy as np
import os

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

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
            #dst = camera_utils.undistort(img, (320, 240))
    
            import torch
    
            with torch.no_grad():
                result = self.model(img)[0]
                image = result.plot()

            b_mask = np.zeros(img.shape[:2], np.uint8)
            for c in result:
                label = c.names[c.boxes.cls.tolist().pop()]
                if label == 'street':
                    contour = c.masks.xy.pop().astype(np.int32).reshape(-1, 1, 2)
                    _ = cv2.drawContours(b_mask, [contour], -1, (255, 255, 255), cv2.FILLED)
            merged_mask = cv2.cvtColor(b_mask, cv2.COLOR_GRAY2BGR)
            blurred_mask = cv2.GaussianBlur(merged_mask, (15, 15), 0)
            img_masked = cv2.bitwise_and(blurred_mask, img)
            edges = cv2.Canny(blurred_mask, 100, 200)

            cv2.line(edges, (0, edges.shape[0]//2), (edges.shape[1], edges.shape[0]//2), (255, 255, 255), 1)            
            contours, _ = cv2.findContours(b_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if len(contours) > 0:
                max_c = max(contours, key=cv2.contourArea)
                M = cv2.moments(max_c)
                if M['m00'] != 0:
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])
                    print(f'distance in y: {cy}')
                    cv2.circle(edges, (cx, cy), 5, (255, 255, 255), -1)
    
            cv2.imshow('Original Image', img)
            cv2.imshow('edges', edges)
            cv2.imshow('street', img_masked)
            cv2.imshow('YOLOv8 Inference', image)
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
                
def main(args=None):
    rclpy.init(args=args)
    camera_node = CameraNode()
    rclpy.spin(camera_node)
    camera_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()