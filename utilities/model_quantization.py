import torch
import torch.nn as nn
from ultralytics import YOLO

model = YOLO('puzzlebot_ws/src/puzzlebot/package_data/best.pt')
model.model.eval()

model.model = torch.quantization.quantize_dynamic(
    model.model,
    {nn.Conv2d, nn.Linear},
    dtype=torch.qint8  # Use 8-bit integers for quantization
)

torch.save(model.model.state_dict(), 'puzzlebot_ws/src/puzzlebot/package_data/best_quantized.pt')

# The quantized model can be loaded and used in the same way as the original model
model_quantized = YOLO('yolov8n-seg.pt')
model_quantized.model.load_state_dict(torch.load('puzzlebot_ws/src/puzzlebot/package_data/best_quantized.pt'))
model_quantized.model.eval()