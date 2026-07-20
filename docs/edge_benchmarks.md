# Edge deployment benchmarks

INT8 dynamic quantization with onnxruntime, measured on the CPU execution provider, the CI and deployment target.

## Health monitor, MLP autoencoder

| Variant | Size KB | Latency ms per window | Fault fires | Nominals silent |
|---|---|---|---|---|
| FP32 | 5.0 | 0.007 | yes | yes |
| INT8 | 7.1 | 0.011 | yes | yes |

Per window error correlation between FP32 and INT8: 0.9999, so the INT8 model keeps the same operating point. The model is tiny, so the quantize and dequantize nodes cost more than they save, INT8 is marginally larger and slower here. This is a parity check, the size win is on the detector below.

## Corrosion detector, YOLOv8n CNN

| Variant | Size KB | Latency ms per frame |
|---|---|---|
| FP32 | 11977.9 | 21.0 |
| INT8 | 3276.9 | 69.1 |

Dynamic quantization stores the convolution weights as INT8, so the file shrinks sharply, see the table. Latency regresses because onnxruntime inserts a dequantize before each convolution on the CPU provider and runs it slower than the tuned FP32 kernel. This is the expected result for dynamic quantization of a convolution network. Turning the smaller model into a faster one needs static QDQ quantization with a calibration set, the documented next step. The mAP50 delta is scored by `scripts/eval_model.py --model models/vision/defect_int8.onnx` against the held out test split, the same tool that scored the FP32 model.

## Detector accuracy on the held out test split

304 images, evaluation confidence 0.001.

| Variant | mAP50 |
|---|---|
| FP32 | 0.621 |
| INT8 | 0.613 |

