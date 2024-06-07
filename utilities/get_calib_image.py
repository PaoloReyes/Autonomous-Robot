import cv2
from camera_demo import gstreamer_pipeline

cap = cv2.VideoCapture(0)
num = 0

while cap.isOpened():

    succes, img = cap.read()
    k = cv2.waitKey(5)
    if k == 27:
        break
    elif k == ord('z'): # wait for 'z' key to save and exit
        cv2.imwrite('images/img' + str(num) + '.png', img)
        print("image saved!")
        num += 1

    cv2.imshow('Img',img)

# Release and destroy all windows before termination
cap.release()

cv2.destroyAllWindows()