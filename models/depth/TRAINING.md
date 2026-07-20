# Depth model

This model estimates monocular depth from one camera image. The depth navigator
uses it to recommend a safer heading when structure is close ahead.

The model is MiDaS small, a pretrained monocular depth network. This project
does not train it, it exports the pretrained weights to ONNX and runs them with
onnxruntime.

## Export the model

The export needs PyTorch. PyTorch is not in the ROS image. Install it one time
in the container as a build tool. A matched torch and torchvision pair is needed
because MiDaS imports timm, which imports torchvision.

Do these steps in the container, from `/ws/src/kestrel`:

1. Install the build tools:

   ```
   pip3 install --break-system-packages torch==2.4.1 torchvision==0.19.1 \
     --index-url https://download.pytorch.org/whl/cpu
   pip3 install --break-system-packages timm onnx "numpy<2"
   ```

2. Run the exporter:

   ```
   python3 scripts/export_depth_model.py
   ```

The exporter downloads the pretrained MiDaS small weights, exports
`models/depth/midas_small.onnx`, and checks that onnxruntime reproduces the
torch depth map. A good result shows a torch to onnxruntime correlation near 1
and the structure reading closer than the sky.

## Preprocessing

The node must match the export preprocessing. The recipe is: convert to RGB,
scale to the range 0 to 1, resize to 256 by 256, then normalize with the mean
`0.485, 0.456, 0.406` and the standard deviation `0.229, 0.224, 0.225`.

## Weights

The ONNX file is 64 MB, too large to commit. Upload it as an asset on a GitHub
release named `depth-model-v1`. The script `models/depth/download_model.sh`
fetches it to `models/depth/midas_small.onnx`, which is gitignored.
