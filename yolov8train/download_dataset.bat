call conda activate ai
python ../utilities/yolov8n-seg_train.py
python ../utilities/coco2yolo.py
call conda deactivate