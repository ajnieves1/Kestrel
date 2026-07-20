#!/usr/bin/env bash
# Fetch the MiDaS small depth ONNX from the GitHub release
set -euo pipefail

MODEL_URL="https://github.com/ajnieves1/Kestrel/releases/download/depth-model-v1/midas_small.onnx"
OUTPUT_PATH="models/depth/midas_small.onnx"

mkdir -p "$(dirname "${OUTPUT_PATH}")"
curl -L "${MODEL_URL}" -o "${OUTPUT_PATH}"
echo "Downloaded ${OUTPUT_PATH}"
