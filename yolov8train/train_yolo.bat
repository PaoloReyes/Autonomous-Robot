call conda activate ai
yolo task=segment mode=train model=yolov8n-seg.pt data=semanticsegmentation-9_yolo\data.yaml batch=16 epochs=25 imgsz=320
