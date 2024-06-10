call conda activate ai
yolo task=segment mode=train model=yolov8n-seg.pt data=semanticsegmentation-5_yolo\data.yaml batch=16 epochs=10 imgsz=320