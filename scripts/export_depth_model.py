#!/usr/bin/env python3
# Export MiDaS_small to ONNX and verify onnxruntime reproduces the depth map.
# Build tool, needs torch. Run in the container:
#   pip3 install --break-system-packages torch==2.4.1 torchvision==0.19.1 \
#     --index-url https://download.pytorch.org/whl/cpu
#   pip3 install --break-system-packages timm "numpy<2"
# See models/depth/TRAINING.md.
import os

import cv2
import numpy as np
import onnxruntime
import torch

MODEL_PATH = 'models/depth/midas_small.onnx'
INPUT_SIZE = 256
# ImageNet normalization, the MiDaS transform, the node must copy this recipe
MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
STD = np.array([0.229, 0.224, 0.225], dtype=np.float32)


# Resize to the model input and normalize into a NCHW float batch
def preprocess(image_bgr):
    rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
    resized = cv2.resize(
        rgb, (INPUT_SIZE, INPUT_SIZE), interpolation=cv2.INTER_CUBIC)
    normalized = (resized - MEAN) / STD
    return np.transpose(normalized, (2, 0, 1))[None].astype(np.float32)


# Export the model and check onnxruntime matches torch on a real frame
def main():
    os.makedirs('models/depth', exist_ok=True)
    midas = torch.hub.load('intel-isl/MiDaS', 'MiDaS_small', trust_repo=True)
    midas.eval()

    dummy = torch.randn(1, 3, INPUT_SIZE, INPUT_SIZE)
    torch.onnx.export(
        midas, dummy, MODEL_PATH,
        input_names=['image'], output_names=['depth'],
        dynamic_axes={'image': {0: 'batch'}, 'depth': {0: 'batch'}},
        opset_version=17)
    print(f'wrote {MODEL_PATH}')

    image = cv2.imread('docs/marker_view.png')
    batch = preprocess(image)
    with torch.no_grad():
        torch_depth = midas(torch.from_numpy(batch)).squeeze().numpy()
    session = onnxruntime.InferenceSession(
        MODEL_PATH, providers=['CPUExecutionProvider'])
    onnx_depth = session.run(None, {'image': batch})[0].squeeze()

    correlation = np.corrcoef(
        torch_depth.flatten(), onnx_depth.flatten())[0, 1]
    print(f'torch to onnxruntime correlation: {correlation:.6f}')

    leg = onnx_depth[80:200, 40:60].mean()
    ground = onnx_depth[210:250, 100:160].mean()
    sky = onnx_depth[10:50, 100:160].mean()
    print(f'onnx depth ordering (larger = closer): '
          f'leg {leg:.1f}  ground {ground:.1f}  sky {sky:.1f}')
    print(f'structure closer than sky: {leg > sky and ground > sky}')


if __name__ == '__main__':
    main()
