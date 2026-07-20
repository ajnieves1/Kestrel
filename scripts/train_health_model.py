#!/usr/bin/env python3
# Train a windowed autoencoder on nominal telemetry and export it to ONNX.
# Runs outside the ROS image, needs torch. onnxruntime is used for the export
# parity check and is already in the image, so run this inside the container
# after a temporary torch install. See models/health/TRAINING.md.
import csv
import glob
import json
import os

import numpy as np
import torch
from torch import nn

FEATURE_COLUMNS = ['voltage', 'current', 'ang_vel_mag', 'lin_acc_mag']
WINDOW_SIZE = 10
SAMPLE_RATE_HZ = 5.0
# A live feature here has std above 0.3, a dead channel has std near 0, so this
# gap cleanly drops dead channels and avoids a divide by zero in scaling
DEAD_FEATURE_STD = 1e-3
THRESHOLD_PERCENTILE = 99.5
EPOCHS = 300
LEARNING_RATE = 1e-3
RANDOM_SEED = 0

LOG_DIRECTORY = 'telemetry_logs'
MODEL_DIRECTORY = 'models/health'
MODEL_PATH = os.path.join(MODEL_DIRECTORY, 'health_model.onnx')
SCALER_PATH = os.path.join(MODEL_DIRECTORY, 'health_scaler.json')


# Read the feature columns of one CSV into an array shaped samples by features
def load_features(path):
    rows = list(csv.DictReader(open(path)))
    return np.array(
        [[float(row[column]) for column in FEATURE_COLUMNS] for row in rows],
        dtype=np.float32)


# Build overlapping windows from one file, flattened, never crossing files
def make_windows(samples, active_index):
    active = samples[:, active_index]
    windows = []
    for start in range(len(active) - WINDOW_SIZE + 1):
        windows.append(active[start:start + WINDOW_SIZE].flatten())
    return np.array(windows, dtype=np.float32)


# A small autoencoder that reconstructs a flattened feature window
class Autoencoder(nn.Module):
    def __init__(self, input_size):
        super().__init__()
        self.encoder = nn.Sequential(
            nn.Linear(input_size, 16), nn.ReLU(),
            nn.Linear(16, 8), nn.ReLU())
        self.decoder = nn.Sequential(
            nn.Linear(8, 16), nn.ReLU(),
            nn.Linear(16, input_size))

    # Encode then decode one batch of windows
    def forward(self, window_batch):
        return self.decoder(self.encoder(window_batch))


# Per window mean squared reconstruction error
def reconstruction_error(model, windows):
    with torch.no_grad():
        reconstructed = model(torch.from_numpy(windows))
        return ((reconstructed - torch.from_numpy(windows)) ** 2).mean(
            dim=1).numpy()


# Train the autoencoder, export ONNX and the scaler, print an offline eval
def main():
    torch.manual_seed(RANDOM_SEED)
    np.random.seed(RANDOM_SEED)

    nominal_paths = sorted(glob.glob(os.path.join(LOG_DIRECTORY, 'nominal_*.csv')))
    if not nominal_paths:
        raise SystemExit('no nominal_*.csv logs found in telemetry_logs')
    print(f'training files: {[os.path.basename(p) for p in nominal_paths]}')

    nominal_files = [load_features(path) for path in nominal_paths]
    all_samples = np.concatenate(nominal_files)

    # Keep only features that actually vary, so dead channels cannot break scaling
    # Statistics in float64, float32 rounding can turn a constant std into noise
    feature_std = all_samples.astype(np.float64).std(axis=0)
    active_index = [i for i in range(len(FEATURE_COLUMNS))
                    if feature_std[i] > DEAD_FEATURE_STD]
    active_names = [FEATURE_COLUMNS[i] for i in active_index]
    means = all_samples[:, active_index].astype(np.float64).mean(axis=0)
    stds = all_samples[:, active_index].astype(np.float64).std(axis=0)
    print(f'active features: {active_names} (dropped '
          f'{[c for c in FEATURE_COLUMNS if c not in active_names]})')

    # Standardize each file, then window it, then stack
    windows = []
    for samples in nominal_files:
        standardized = samples.copy()
        standardized[:, active_index] = (
            samples[:, active_index] - means) / stds
        windows.append(make_windows(standardized, active_index))
    training_windows = np.concatenate(windows)
    input_size = WINDOW_SIZE * len(active_index)
    print(f'training windows: {len(training_windows)}, input size {input_size}')

    model = Autoencoder(input_size)
    optimizer = torch.optim.Adam(model.parameters(), lr=LEARNING_RATE)
    loss_function = nn.MSELoss()
    window_tensor = torch.from_numpy(training_windows)

    model.train()
    for epoch in range(EPOCHS):
        optimizer.zero_grad()
        loss = loss_function(model(window_tensor), window_tensor)
        loss.backward()
        optimizer.step()
        if (epoch + 1) % 50 == 0:
            print(f'epoch {epoch + 1}/{EPOCHS} loss {loss.item():.6f}')

    model.eval()
    nominal_errors = reconstruction_error(model, training_windows)
    threshold = float(np.percentile(nominal_errors, THRESHOLD_PERCENTILE))
    print(f'nominal error: mean {nominal_errors.mean():.6f} '
          f'p{THRESHOLD_PERCENTILE} {threshold:.6f} max {nominal_errors.max():.6f}')

    os.makedirs(MODEL_DIRECTORY, exist_ok=True)
    # Dynamic batch axis so the node feeds one window and the eval feeds many
    torch.onnx.export(
        model, window_tensor[:1], MODEL_PATH,
        input_names=['window'], output_names=['reconstruction'],
        dynamic_axes={'window': {0: 'batch'}, 'reconstruction': {0: 'batch'}},
        opset_version=17, dynamo=False)

    scaler = {
        'feature_names': active_names,
        'means': means.tolist(),
        'stds': stds.tolist(),
        'window_size': WINDOW_SIZE,
        'sample_rate_hz': SAMPLE_RATE_HZ,
        'threshold': threshold,
    }
    with open(SCALER_PATH, 'w') as scaler_file:
        json.dump(scaler, scaler_file, indent=2)
    print(f'wrote {MODEL_PATH} and {SCALER_PATH}')

    offline_eval(active_index, means, stds, threshold)


# Score the ONNX model with onnxruntime against nominal and fault logs
def offline_eval(active_index, means, stds, threshold):
    import onnxruntime

    session = onnxruntime.InferenceSession(MODEL_PATH)
    input_name = session.get_inputs()[0].name

    def over_threshold_fraction(path):
        samples = load_features(path)
        samples[:, active_index] = (samples[:, active_index] - means) / stds
        windows = make_windows(samples, active_index)
        if len(windows) == 0:
            return 0.0, 0
        outputs = session.run(None, {input_name: windows})[0]
        errors = ((outputs - windows) ** 2).mean(axis=1)
        return float((errors > threshold).mean()), len(windows)

    print('\noffline eval (fraction of windows over threshold):')
    for path in sorted(glob.glob(os.path.join(LOG_DIRECTORY, '*.csv'))):
        fraction, count = over_threshold_fraction(path)
        print(f'  {os.path.basename(path):40s} {fraction * 100:6.1f}%  '
              f'({count} windows)')


if __name__ == '__main__':
    main()
