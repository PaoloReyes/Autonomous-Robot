call conda activate ai
yolo task=segment mode=train model=yolov8n-seg.pt data=semanticsegmentation-7_yolo\data.yaml batch=16 epochs=5 imgsz=320