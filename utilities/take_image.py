import cv2
from camera_demo import gstreamer_pipeline
from time import sleep

global PHOTO_NUM
# Change the value to the desired photo number
PHOTO_NUM = 0

def main():
    cap = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
    for i in range(PHOTO_NUM):
        try:
            _, frame = cap.read()
            cv2.imwrite(f'images/image_{i}.png', frame)
            sleep(0.5) # Delay for half second
        except:
            print('Failed to capture image')

    cap.release()

if __name__ == '__main__':
    main()