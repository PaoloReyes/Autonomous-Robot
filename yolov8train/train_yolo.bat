call conda activate ai
yolo task=segment mode=train model=yolov8n-seg.pt data=semanticsegmentation-10_yolo\data.yaml batch=16 epochs=100 imgsz=320 close_mosaic=15 fliplr=0.0 scale=0.0 iou=0.4