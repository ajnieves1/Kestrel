#!/usr/bin/env python3
# Score the shipped ONNX model against the dataset's held out test split.
# Inference mirrors defect_detector.py exactly: letterbox, BGR to RGB,
# normalize, NMS, then undo the letterbox back to original pixel space.
import argparse
import io
import os

import cv2
import numpy as np
import pandas as pd
import onnxruntime
from PIL import Image

YOLO_INPUT_SIZE = 640
YOLO_NMS_IOU_THRESHOLD = 0.45
MATCH_IOU_THRESHOLD = 0.5

TEST_PARQUET_URL = (
    'https://huggingface.co/api/datasets/Francesco/corrosion-bi3q3'
    '/parquet/default/test/0.parquet')


# Resize and pad a frame to a square YOLO input, keeping the aspect ratio
def letterbox_image(cv_image):
    height, width = cv_image.shape[:2]
    scale = min(YOLO_INPUT_SIZE / height, YOLO_INPUT_SIZE / width)
    new_height, new_width = round(height * scale), round(width * scale)
    resized = cv2.resize(cv_image, (new_width, new_height))

    pad_left = (YOLO_INPUT_SIZE - new_width) // 2
    pad_top = (YOLO_INPUT_SIZE - new_height) // 2
    pad_right = YOLO_INPUT_SIZE - new_width - pad_left
    pad_bottom = YOLO_INPUT_SIZE - new_height - pad_top
    padded = cv2.copyMakeBorder(
        resized, pad_top, pad_bottom, pad_left, pad_right,
        cv2.BORDER_CONSTANT, value=(114, 114, 114))

    return padded, scale, pad_left, pad_top


# Run the model on one BGR frame, return corner boxes with confidences
def run_yolo(session, input_name, cv_image, confidence_threshold):
    letterboxed, scale, pad_left, pad_top = letterbox_image(cv_image)
    rgb_image = cv2.cvtColor(letterboxed, cv2.COLOR_BGR2RGB)
    normalized = rgb_image.astype(np.float32) / 255.0
    input_tensor = np.transpose(normalized, (2, 0, 1))[np.newaxis, :]

    outputs = session.run(None, {input_name: input_tensor})
    predictions = outputs[0][0].T

    boxes = []
    scores = []
    for prediction in predictions:
        confidence = float(prediction[4:].max())
        if confidence < confidence_threshold:
            continue
        center_x, center_y, width, height = prediction[:4]
        boxes.append([
            float(center_x - width / 2), float(center_y - height / 2),
            float(width), float(height)])
        scores.append(confidence)

    if not boxes:
        return []

    kept_indices = cv2.dnn.NMSBoxes(
        boxes, scores, confidence_threshold, YOLO_NMS_IOU_THRESHOLD)

    results = []
    for index in np.array(kept_indices).flatten():
        x, y, width, height = boxes[index]
        x = (x - pad_left) / scale
        y = (y - pad_top) / scale
        width = width / scale
        height = height / scale
        results.append((x, y, x + width, y + height, scores[index]))

    return results


# Intersection over union of two corner boxes
def box_iou(box_a, box_b):
    left = max(box_a[0], box_b[0])
    top = max(box_a[1], box_b[1])
    right = min(box_a[2], box_b[2])
    bottom = min(box_a[3], box_b[3])
    if right <= left or bottom <= top:
        return 0.0

    intersection = (right - left) * (bottom - top)
    area_a = (box_a[2] - box_a[0]) * (box_a[3] - box_a[1])
    area_b = (box_b[2] - box_b[0]) * (box_b[3] - box_b[1])
    return intersection / (area_a + area_b - intersection)


# Average precision as the area under the precision recall curve
def average_precision(detections, total_ground_truth):
    detections.sort(key=lambda detection: -detection[0])
    true_positives = 0
    false_positives = 0
    precisions = []
    recalls = []
    for _, is_match in detections:
        if is_match:
            true_positives += 1
        else:
            false_positives += 1
        precisions.append(true_positives / (true_positives + false_positives))
        recalls.append(true_positives / total_ground_truth)

    # Make precision monotonically decreasing, then sum the recall steps
    for index in range(len(precisions) - 2, -1, -1):
        precisions[index] = max(precisions[index], precisions[index + 1])

    area = 0.0
    previous_recall = 0.0
    for precision, recall in zip(precisions, recalls):
        area += (recall - previous_recall) * precision
        previous_recall = recall

    return area


# Evaluate every test image and return scored detections plus totals
def evaluate(session, input_name, dataframe, confidence_threshold):
    scored_detections = []
    total_ground_truth = 0

    for _, row in dataframe.iterrows():
        pil_image = Image.open(io.BytesIO(row['image']['bytes'])).convert('RGB')
        cv_image = cv2.cvtColor(np.array(pil_image), cv2.COLOR_RGB2BGR)

        ground_truth = [
            (box[0], box[1], box[0] + box[2], box[1] + box[3])
            for box in row['objects']['bbox']]
        total_ground_truth += len(ground_truth)

        detections = run_yolo(session, input_name, cv_image, confidence_threshold)
        detections.sort(key=lambda detection: -detection[4])

        matched = set()
        for x1, y1, x2, y2, confidence in detections:
            best_iou = 0.0
            best_index = None
            for index, truth_box in enumerate(ground_truth):
                if index in matched:
                    continue
                iou = box_iou((x1, y1, x2, y2), truth_box)
                if iou > best_iou:
                    best_iou = iou
                    best_index = index
            is_match = best_iou >= MATCH_IOU_THRESHOLD
            if is_match:
                matched.add(best_index)
            scored_detections.append((confidence, is_match))

    return scored_detections, total_ground_truth


# Parse arguments, run the evaluation, and print the metrics
def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', default='models/vision/defect.onnx')
    parser.add_argument('--parquet', default='test.parquet',
                        help=f'Test split parquet, download from {TEST_PARQUET_URL}')
    parser.add_argument('--eval-confidence', type=float, default=0.001,
                        help='Low threshold so the full precision recall curve exists')
    parser.add_argument('--report-confidence', type=float, default=0.5,
                        help='Production threshold to report precision and recall at')
    arguments = parser.parse_args()

    if not os.path.isfile(arguments.model):
        raise SystemExit(f'{arguments.model} missing, run models/vision/download_model.sh')

    session = onnxruntime.InferenceSession(
        arguments.model, providers=['CPUExecutionProvider'])
    input_name = session.get_inputs()[0].name
    dataframe = pd.read_parquet(arguments.parquet)

    scored_detections, total_ground_truth = evaluate(
        session, input_name, dataframe, arguments.eval_confidence)

    map50 = average_precision(list(scored_detections), total_ground_truth)

    kept = [d for d in scored_detections if d[0] >= arguments.report_confidence]
    true_positives = sum(1 for _, is_match in kept if is_match)
    precision = true_positives / len(kept) if kept else 0.0
    recall = true_positives / total_ground_truth if total_ground_truth else 0.0

    print(f'images: {len(dataframe)}, ground truth boxes: {total_ground_truth}')
    print(f'mAP50: {map50:.3f}')
    print(f'at confidence {arguments.report_confidence}: '
          f'precision {precision:.3f}, recall {recall:.3f} '
          f'({true_positives}/{len(kept)} detections matched)')


if __name__ == '__main__':
    main()
