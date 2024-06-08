import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
from .submodules import camera_utils
import torch
import os
from ament_index_python import get_package_share_directory

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_segmentation_node')
        self.bridge = CvBridge()
        
        # Publisher
        #self.pub = self.create_publisher(Image, '/raw_camera', 10)
        self.pub = self.create_publisher(Bool, 'ready', 10)

        # Open the camera feed
        print(self.gstreamer_pipeline(flip_method=0))
        self.source = cv2.VideoCapture(self.gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)

        from ultralytics import YOLO

        # Timers
        timer_period = 0.01 
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
            msg = Bool()
            msg.data = True
            cv2.imshow('Camera Feed', dst)
            with torch.no_grad():
                dst = dst.to(device= self.device, dtype=torch.float32)
                result = self.model(dst)
            image = result[0].plot()

            cv2.imshow('YOLOv8 Inference', image)
            cv2.waitKey(1)
            self.pub.publish(msg)
            #msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
            #self.pub.publish(msg)

        else:
            print('Unable to open camera')
    
    def gstreamer_pipeline(self, sensor_id=0, capture_width=320, capture_height=240, display_width=320, display_height=240,framerate=30, flip_method=0):
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