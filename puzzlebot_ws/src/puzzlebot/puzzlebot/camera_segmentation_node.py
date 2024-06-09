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

            mid_images = edges.copy()
            cv2.line(mid_images, (0, mid_images.shape[0]//2), (mid_images.shape[1], mid_images.shape[0]//2), (255, 255, 255), 1)

            b_mask = cv2.cvtColor(b_mask, cv2.COLOR_BGR2GRAY)
            mid = b_mask[b_mask.shape[0]//2:,:]

            # Convert the image to grayscale if it is not already
            if len(mid.shape) == 3 and mid.shape[2] == 3:
                mid = cv2.cvtColor(mid, cv2.COLOR_BGR2GRAY)
            
            [contours, _] = cv2.findContours(mid.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if len(contours) > 0:
                cx_t = 0
                cy_t = 0
                i = 0
                for cnt in contours:
                    M = cv2.moments(cnt)
                    if M['m00'] != 0:
                        cx = int(M['m10']/M['m00'])
                        cy = int(M['m01']/M['m00'])
                        cy += mid_images.shape[0]//2
                        cx_t += cx
                        cy_t += cy
                        i += 1
                        cv2.circle(mid_images, (cx, cy), 5, (255, 255, 255), -1)
                if i != 0:
                    cx_t = cx_t // i
                    cy_t = cy_t // i
                    cv2.circle(mid_images, (cx_t, cy_t), 5, (255, 255, 255), -1)
    
            cv2.imshow('Original Image', img)
            cv2.imshow('edges', edges)
            cv2.imshow('street', img_masked)
            cv2.imshow('YOLOv8 Inference', image)
            cv2.imshow('pussy controller', mid_images)
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