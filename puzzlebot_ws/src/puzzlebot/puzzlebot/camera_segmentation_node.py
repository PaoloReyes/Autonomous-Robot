import cv2
import numpy as np
import os

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from cv_bridge import CvBridge

from .submodules import camera_utils

from ament_index_python import get_package_share_directory

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_segmentation_node')
        self.bridge = CvBridge()
        
        # Publisher
        #self.pub = self.create_publisher(Image, '/raw_camera', 10)
        self.pub = self.create_publisher(String, 'debug', 10)

        # Open the camera feed
        print(self.gstreamer_pipeline(flip_method=0))
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
        if torch.cuda.is_available():
            self.device = torch.device('cuda')
        else:
            self.device = torch.device('cpu')
        print('Using device:', self.device)
        self.model.model.eval()
        self.model.to(self.device)

    def timer_callback(self):
        if self.source.isOpened():
            _, img = self.source.read()
            dst = camera_utils.undistort(img, (320, 240))
            cv2.imshow('Original Image', dst)
    
            import torch
    
            with torch.no_grad():
                result = self.model(dst)[0]
                image = result.plot()
                img_copy = np.copy(result.orig_img)

            for c in result:
                label = c.names[c.boxes.cls.tolist().pop()]
                b_mask = np.zeros(img.shape[:2], np.uint8)
                contour = c.masks.xy.pop()
                contour = contour.astype(np.int32)
                contour = contour.reshape(-1, 1, 2)
                _ = cv2.drawContours(b_mask, [contour], -1, (255, 255, 255), cv2.FILLED)
                mask3ch = cv2.cvtColor(b_mask, cv2.COLOR_GRAY2BGR)
                isolated_mask = cv2.bitwise_and(mask3ch, img_copy)
                cv2.imshow(label, isolated_mask)

            msg = String()
            msg.data = result.verbose()

            cv2.imshow('YOLOv8 Inference', image)
            cv2.waitKey(1)
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