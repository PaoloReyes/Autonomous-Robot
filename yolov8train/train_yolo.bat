call conda activate ai
yolo task=segment mode=train model=yolov8n-seg.pt data=semanticsegmentation-3_yolo\data.yaml batch=16 epochs=30 imgsz=320