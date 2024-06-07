import cv2
import numpy as np
import pickle
from camera_demo import gstreamer_pipeline

def main():
    print("Starting fisheye calibration...")

    # Define the dimensions of the checkerboard
    CHECKERBOARD = (6, 8)

    # Define the camera for capturing images
    cap = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)

    # Define the number of photos to take
    N_PHOTO = 140

    criteria_pix = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)

    # Create arrays to store object points and image points from all the images
    objpoints = []
    imgpoints = []

    # Prepare the object points, like (0,0,0), (1,0,0), (2,0,0), ..., (6,5,0)
    objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
    objp[0, :, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)

    for i in range(N_PHOTO):
        _, frame = cap.read()
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Find the chessboard corners
        ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)
        print('Capturing image', i, '...')
        cv2.imshow('Capturing image', frame)

        if ret:
            objpoints.append(objp)
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria_pix)
            imgpoints.append(corners2)

            # Draw and display the corners
            img = cv2.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)
            cv2.imshow('Checkerboard', img)
            cv2.waitKey(500)

    cv2.destroyAllWindows()
    cap.release()

    # Perform fisheye calibration
    K = np.zeros((3, 3))
    D = np.zeros((4, 1))
    rvecs = []
    tvecs = []

    # Define calibration flags
    calibration_flags = (cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC +
                        cv2.fisheye.CALIB_CHECK_COND +
                        cv2.fisheye.CALIB_FIX_SKEW)

    # Calibrate
    try:
        rms, K, D, rvecs, tvecs = cv2.fisheye.calibrate(
            objpoints,
            imgpoints,
            gray.shape[::-1],
            K,
            D,
            rvecs,
            tvecs,
            calibration_flags,
            criteria
        )

        print("Calibration was successful.")
        print("RMS:", rms)
        print("K:", K)
        print("D:", D)

        # Save calibration results using pickle
        with open('calibration_data.pkl', 'wb') as f:
            pickle.dump((K, D, rvecs, tvecs), f)
        print("Calibration data saved to calibration_data.pkl.")

    except cv2.error as e:
        print("Calibration failed: ", e)

if __name__ == '__main__':
    main()