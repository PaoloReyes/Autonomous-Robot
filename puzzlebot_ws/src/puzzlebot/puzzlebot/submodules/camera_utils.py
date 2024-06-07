import cv2
import numpy as np
import pickle

with open('calibration_data.pkl', 'rb') as f:
    K, D, _, _ = pickle.load(f)
print('Calibration data loaded from calibration_data.pkl')

def undistort(img, DIM):
    global K, D
    h,w = img.shape[:2]
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)
    undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    return undistorted_img