import cv2
import numpy as np
import pickle
from ament_index_python import get_package_share_directory
import os

path = get_package_share_directory('puzzlebot')
path = path.split('install')[0]
path = os.path.join(path, 'src', 'puzzlebot','package_data', 'calibration_data.pkl')

with open(path, 'rb') as f:
    K, D, _, _ = pickle.load(f)
print('Calibration data loaded from calibration_data.pkl')

def undistort(img, DIM):
    global K, D
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)
    undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    return undistorted_img