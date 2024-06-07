import torch
import torch.nn as nn
import cv2
from ultralytics import YOLO
import numpy as np
import matplotlib.pyplot as plt


def main():
    model = YOLO('best.pt')
    device = torch.device('cuda') if torch.cuda.is_available() else torch.device('cpu')
    model.to(device)

    frame = cv2.VideoCapture(0)

    while True:
        _, origin = frame.read()
        #img = cv2.cvtColor(origin, cv2.COLOR_BGR2RGB)
        results = model(origin)

        image = results[0].cpu().plot(boxes=None)
        probs = results[0].masks
        cv2.imshow('YOLOv8 Inference', image)
        cv2.imshow('Original', origin)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        
    frame.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()