call conda activate ai
yolo task=segment mode=train model=yolov8n-seg.pt data=semanticsegmentation-17_yolo\data.yaml batch=16 epochs=50 imgsz=320 close_mosaic=25 fliplr=0.0 scale=0.0 iou=0.5