#!/usr/bin/env bash
# Fetch the trained corrosion detector ONNX weights from the GitHub release
set -euo pipefail

MODEL_URL="https://github.com/ajnieves1/Kestrel/releases/download/defect-model-v1/defect.onnx"
OUTPUT_PATH="models/vision/defect.onnx"

mkdir -p "$(dirname "${OUTPUT_PATH}")"
curl -L "${MODEL_URL}" -o "${OUTPUT_PATH}"
echo "Downloaded ${OUTPUT_PATH}"
