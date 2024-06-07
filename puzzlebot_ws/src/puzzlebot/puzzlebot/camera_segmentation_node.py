import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_segmentation_node')
        self.bridge = CvBridge()
        
        # Publisher
        self.pub = self.create_publisher(Image, '/raw_camera', 10)

        # Open the camera feed
        self.source = cv2.VideoCapture(self.gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
        
        # Timers
        timer_period = 0.0333
        self.timer = self.create_timer(timer_period, self.timer_callback)
    
    def timer_callback(self):
        if self.source.isOpened():
            _, img = self.source.read()
            self.get_logger().info(type(img))
            self.pub.publish(self.bridge.cv2_to_imgmsg(img, 'bgr8'))
        else:
            print('Unable to open camera')
    
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
    camera_node = CameraNode()
    rclpy.spin(camera_node)
    camera_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()