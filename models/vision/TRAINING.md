# Training the corrosion detector

The data set is
[`Francesco/corrosion-bi3q3`](https://huggingface.co/datasets/Francesco/corrosion-bi3q3)
on Hugging Face, part of the Roboflow 100 benchmark. This data set
mirrors
[this Roboflow Universe project](https://universe.roboflow.com/khaingwintz-gmail-com/dataset--2-pathein-train-plus-v-3-update-mm)
under a CC BY 4.0 license. The data set has 1249 images: 840 for
training, 105 for validation, and 304 for the test split. Each image has
bounding box annotations. The original data set has four category
labels. Training used one class only, `corrosion`, so this process
combined all four labels into that one class.

## Environment

```bash
python3 -m venv yolo_venv
source yolo_venv/bin/activate
pip install ultralytics pandas pyarrow pillow onnx onnxruntime
```

## Fetch and convert the data set

This process downloads the three parquet splits directly over HTTP. No
account and no API key are necessary.

```bash
curl -L "https://huggingface.co/api/datasets/Francesco/corrosion-bi3q3/parquet/default/train/0.parquet" -o train.parquet
curl -L "https://huggingface.co/api/datasets/Francesco/corrosion-bi3q3/parquet/default/validation/0.parquet" -o validation.parquet
curl -L "https://huggingface.co/api/datasets/Francesco/corrosion-bi3q3/parquet/default/test/0.parquet" -o test.parquet
```

Each row holds one image, its width, its height, and a list of `bbox`
entries. Each `bbox` entry uses `[x, y, w, h]` pixel coordinates. The
conversion step writes YOLO format label files. Each label file uses
normalized `class cx cy w h` values. The conversion step maps every
category ID to class `0`, `corrosion`.

`dataset.yaml`:

```yaml
path: /path/to/corrosion_yolo
train: images/train
val: images/val
test: images/test
names:
  0: corrosion
```

## Train

```bash
yolo detect train model=yolov8n.pt data=dataset.yaml imgsz=640 epochs=50 batch=16 device=cpu project=runs name=corrosion_yolov8n
```

This training run used CPU only, on a Ryzen 9 5900X processor with 24
threads. Each epoch took about 2 minutes. The full run of 50 epochs took
about 1.6 hours.

## Export

```bash
yolo export model=runs/corrosion_yolov8n/weights/best.pt format=onnx imgsz=640
```

The exported ONNX model takes an input of shape `[1, 3, 640, 640]`. It
produces an output of shape `[1, 5, 8400]`.

## Result

The training run completed all 50 epochs. On the validation split, the
model reached an mAP50 score of 0.617, a precision of 0.867, and a recall
of 0.571. These numbers come from real photos, not from the simulation.

The detector does not reliably find defects inside the Gazebo
simulation. This is a known limitation, not a training error. See
[docs/benchmarks.md](../../docs/benchmarks.md) in the repository root for
the measured numbers from live mission runs.
