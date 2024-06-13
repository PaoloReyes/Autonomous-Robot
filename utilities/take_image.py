import cv2
from camera_demo import gstreamer_pipeline
from time import sleep

global PHOTO_NUM
# Change the value to the desired photo number
PHOTO_NUM = 10

def main():
    cap = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
    i = 0
    while True:
        try:
            _, frame = cap.read()
            frame = cv2.flip(frame, -1)
            cv2.imwrite(f'images/image_{i}.png', frame)
            print(f'Captured image {i}')
            i += 1
            sleep(0.5) # Delay for half second
        except:
            print('Failed to capture image')

    cap.release()
    print('Done capturing images')

if __name__ == '__main__':
    main()