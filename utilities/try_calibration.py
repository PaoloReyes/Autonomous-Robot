import cv2
import numpy as np
import pickle
from camera_demo import gstreamer_pipeline

def undistort(img, K, D, DIM):
    h,w = img.shape[:2]
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)
    undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    return undistorted_img

def main():
    with open('calibration_data.pkl', 'rb') as f:
        K, D, rvecs, tvecs = pickle.load(f)

    print("K:", K)
    print("D:", D)

    cap = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
    while True:
        _, frame = cap.read()
        udst = undistort(frame, K, D, frame.shape[:2])
        cv2.imshow("undistorted", udst)
        cv2.imshow("original", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()
    cap.release()

if __name__ == "__main__":
    main()