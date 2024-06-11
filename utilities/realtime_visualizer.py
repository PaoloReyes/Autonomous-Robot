import torch
import torch.nn as nn

import cv2

import numpy as np
import matplotlib.pyplot as plt

from ultralytics import YOLO

def main():
    MODEL_PATH = r'D:\rokur\Documents\ITESM\6to\Autonomous-Robot\yolov8train\runs\segment\train25\weights\best.pt'
    
    model = YOLO(MODEL_PATH)
    device = torch.device('cuda') if torch.cuda.is_available() else torch.device('cpu')
    model.to(device)

    camera = cv2.VideoCapture(0)

    while True:
        _, origin = camera.read()
        results = model(origin)

        image = results[0].cpu().plot()
        cv2.imshow('YOLOv8 Inference', image)
        cv2.imshow('Original', origin)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        
    camera.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()