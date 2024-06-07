import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.bridge = CvBridge()
        self.image_width = 640
        self.image_height = 480
        self.img = np.ndarray((self.image_height, self.image_width, 3))
        
        self.pub = self.create_publisher(Image, 'Camera', 10)
        # Timers
        timer_period = 1/3 # Frames
        self.timer = self.create_timer(timer_period, self.timer_callback)
    
    def timer_callback(self):
        self.img = cv2.VideoCapture(self.gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
        if self.img.isOpened():
            self.pub.publish(self.bridge.cv2_to_imgmsg(self.img, 'bgr8'))   
        else:
            print('Unable to open camera')
        
    # Create a Camera class that will be used to show the camera feed
    def gstreamer_pipeline(self, capture_width=320, capture_height=240, display_width=320, display_height=240, framerate=30, flip_method=0):
        return (
            "nvarguscamerasrc ! "
            "video/x-raw(memory:NVMM), "
            "width=(int)%d, height=(int)%d, "
            "format=(string)NV12, framerate=(fraction)%d/1 ! "
            "nvvidconv flip-method=%d ! "
            "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
            "videoconvert ! "
            "video/x-raw, format=(string)BGR ! appsink"
            % (capture_width, capture_height, framerate, flip_method, display_width, display_height)
        )
              
def main(args=None):
    rclpy.init(args=args)
    camera_node = CameraSubscriber()
    rclpy.spin(camera_node)
    camera_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()