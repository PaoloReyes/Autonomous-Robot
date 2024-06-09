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

            # search the start point of each line of street
            start_point_1, start_point_2 = self.find_start_points(edges)
            
            if start_point_1 is not None and start_point_2 is not None:
                lines = [[], []]
                lines[0].append(start_point_1)
                lines[1].append(start_point_2)
                
                self.points = []
                self.run_line(edges, actual_coord=start_point_1, i=0)
                lines[0].append(np.array(self.points))
                
                rkn254 = np.zeros(img.shape[:2], np.uint8)
                for line in lines:
                    if line:
                        contour = line
                        _ = cv2.drawContours(rkn254, [contour], -1, (0, 255, 0), cv2.F)
            

                rkn254 = np.zeros(img.shape[:2], np.uint8)
                # Drawing the lines
                for line in lines:
                    i = 0
                    contour = line
                    if i == 0:
                        i += 1
                        _ = cv2.drawContours(rkn254, [contour], -1, (0, 255, 0), cv2.FILLED)
                    else:
                        _ = cv2.drawContours(rkn254, [contour], -1, (0, 0, 255), cv2.FILLED)

            cv2.imshow('Original Image', dst)
            cv2.imshow('edges', edges)
            cv2.imshow('street', img_masked)
            cv2.imshow('YOLOv8 Inference', image)
            #cv2.imshow('rkn254', rkn254)
            cv2.waitKey(1)

            msg = String()
            msg.data = result.verbose()

            self.pub.publish(msg)
            #msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
            #self.pub.publish(msg)

        else:
            print('Unable to open camera')

    def find_start_points(self, edges):
        start_point_1 = None
        start_point_2 = None
        for y in range(edges.shape[0] - 1, -1, -1):
            for x in range(edges.shape[1]):
                if edges[y, x] == 255:
                    start_point_1 = (x, y)
                    break
            if start_point_1:
                for x in range(edges.shape[1] - 1, -1, -1):
                    if edges[y, x] == 255:
                        start_point_2 = (x, y)
                        break
            if start_point_1 and start_point_2:
                if start_point_1[0] != start_point_2[0]:
                    break
                else:
                    start_point_1 = None
                    start_point_2 = None
        return start_point_1, start_point_2

    def merge_two_lines(self, line1, line2):
        x1, y1, x2, y2 = line1
        x3, y3, x4, y4 = line2

        avg_start_x = (x1 + x3) / 2
        avg_start_y = (y1 + y3) / 2
        avg_end_x = (x2 + x4) / 2
        avg_end_y = (y2 + y4) / 2

        return (int(avg_start_x), int(avg_start_y), int(avg_end_x), int(avg_end_y))

    def merge_lines(self, lines, threshold_distance=10, threshold_angle=10):
        def calculate_angle(line):
            x1, y1, x2, y2 = line
            angle = np.degrees(np.arctan2(y2 - y1, x2 - x1))
            return angle

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
                merged_line = self.merge_two_lines(merged_line, line)
            merged_lines.append(merged_line)

        return merged_lines
    
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

    def run_line(self, edges, past_coord=None, actual_coord=None, i=0):
        if i == 240:
            return past_coord
        if past_coord is None:
            past_coord = (None, None)
        if actual_coord is None:
            actual_coord = (None, None)
        
        x_past, y_past = past_coord
        x_act, y_act = actual_coord
        
        for dx in range(-1, 2):
            for dy in range(-1, 2):
                x2, y2 = x_act + dx, y_act + dy
                if 0 <= x2 < edges.shape[1] and 0 <= y2 < edges.shape[0] and edges[y2, x2] == 255:
                    if (x2, y2) != (x_past, y_past) and (x2, y2) != (x_act, y_act):
                        print(f'i: {i}, x2: {x2}, y2: {y2}')
                        self.points.append((x2, y2))
                        return self.run_line(edges, actual_coord, (x2, y2), i + 1)
        
        return past_coord
                
                
def main(args=None):
    rclpy.init(args=args)
    camera_node = CameraNode()
    rclpy.spin(camera_node)
    camera_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()