#!/usr/bin/env python3
from ultralytics import YOLO
import torch
import sys

# 1) Point to your classifier weights
WEIGHTS = '/home/dadn/wc_ws/src/camera/yolobot_recognition/scripts/Classi_YOLOv8.pt'

# 2) Load model
model = YOLO(WEIGHTS)

# 3) Print the names dict
print(">>> model.names dict:")
for idx, name in model.names.items():
    print(f"  {idx:>2} → {name}")

# 4) Print summary info
print("\n>>> model.info():")
model.info()

# 5) (Optional) Inspect checkpoint directly
ckpt = torch.load(WEIGHTS, map_location='cpu')
if 'model' in ckpt and hasattr(ckpt['model'], 'names'):
    print("\n>>> checkpoint['model'].names:")
    for idx, name in ckpt['model'].names.items():
        print(f"  {idx:>2} → {name}")
else:
    print("\n>>> no 'model.names' found in checkpoint")
