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
            dst = camera_utils.undistort(img, (320, 240))
    
            import torch
    
            with torch.no_grad():
                result = self.model(dst)[0]
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
            lines = cv2.HoughLinesP(edges, 1, np.pi/180, 35, maxLineGap=100)
            if lines is not None: lines = self.merge_lines(lines.reshape(-1, 4), threshold_distance=20, threshold_angle=10)

            if lines is not None:
                group = [[], [], []]
                coord_group = [[], [], []]
                for line in lines:
                    x1, y1, x2, y2 = line
                    m = self.get_m(x1, y1, x2, y2)
                    # The line is 'vertical'
                    if m > 0.8:
                        cv2.line(dst, (x1, y1), (x2, y2), (241, 111, 188), 2)
                        group[0].append((x1,y1, x2, y2))
                    
                    # The line is 'diagonal'
                    elif m > 0.3 and m < 0.8:
                        cv2.line(dst, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        group[1].append((x1,y1, x2, y2))
                    
                    # The line is 'horizontal'
                    else:
                        cv2.line(dst, (x1, y1), (x2, y2), (255, 0, 0), 2)
                        group[2].append((x1,y1, x2, y2))

                print(group)

                for i in range(len(group)):
                    if len(group[i]) > 1:
                        for k in range(len(group[i])):
                            coord_group[i] = self.merge_two_lines(group[i][k], coord_group[i])
                    else:
                        coord_group[i] = group[i]

                for i in range(len(coord_group)):
                    if len(coord_group[i]) > 0:
                        x1, y1, x2, y2 = coord_group[i]
                        cv2.line(dst, (x1, y1), (x2, y2), (15, 219, 133), 2)

            cv2.imshow('Original Image', dst)
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

    def merge_lines(self, lines, threshold_distance=10, threshold_angle=10):
        def calculate_angle(line):
            x1, y1, x2, y2 = line
            angle = np.degrees(np.arctan2(y2 - y1, x2 - x1))
            return angle

        def merge_two_lines(line1, line2):
            x1, y1, x2, y2 = line1
            x3, y3, x4, y4 = line2

            avg_start_x = (x1 + x3) / 2
            avg_start_y = (y1 + y3) / 2
            avg_end_x = (x2 + x4) / 2
            avg_end_y = (y2 + y4) / 2

            return (int(avg_start_x), int(avg_start_y), int(avg_end_x), int(avg_end_y))

        # Group lines
        groups = []
        for line in lines:
            added_to_group = False
            for group in groups:
                for member in group:
                    angle1 = calculate_angle(line)
                    angle2 = calculate_angle(member)
                    if abs(angle1 - angle2) < threshold_angle:
                        distance = np.linalg.norm(np.array(line[:2]) - np.array(member[:2]))
                        if distance < threshold_distance:
                            group.append(line)
                            added_to_group = True
                            break
                if added_to_group:
                    break
            if not added_to_group:
                groups.append([line])

        # Merge lines in each group
        merged_lines = []
        for group in groups:
            merged_line = group[0]
            for line in group[1:]:
                merged_line = merge_two_lines(merged_line, line)
            merged_lines.append(merged_line)

        return merged_lines
    
    def get_m(self, x1, y1, x2, y2):
        m = (y2 - y1) / (x2 - x1)
        return np.abs(m)

    def merge_two_lines(self, line1, line2):
            x1, y1, x2, y2 = line1
            x3, y3, x4, y4 = line2

            avg_start_x = (x1 + x3) / 2
            avg_start_y = (y1 + y3) / 2
            avg_end_x = (x2 + x4) / 2
            avg_end_y = (y2 + y4) / 2

            return (int(avg_start_x), int(avg_start_y), int(avg_end_x), int(avg_end_y))

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