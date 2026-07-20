# Health monitor model

This model finds propulsion anomalies from flight telemetry. It learns a
healthy flight. It then flags telemetry that does not match.

## Features

The recorder reads four features at 5 Hz:

- `voltage`: battery voltage in volts.
- `current`: battery current in amps.
- `ang_vel_mag`: the magnitude of the angular velocity in radians per second.
- `lin_acc_mag`: the magnitude of the linear acceleration in meters per second
  squared.

A weak motor changes these features together. The current goes up. The
vibration goes up. The autopilot works harder to hold the attitude.

## Record nominal data

Do these steps to record a healthy flight:

1. Start the flight stack.
2. Run the recorder with the label `nominal`:

   ```
   python3 scripts/record_telemetry.py --ros-args -p label:=nominal
   ```

3. Fly one full mission.
4. Stop the recorder with Ctrl-C after the vehicle lands.

The recorder writes the file `telemetry_logs/nominal_<timestamp>.csv`. It
records rows only while the vehicle is armed.

## Record fault data

Do these steps to record a flight with a weak motor. This data is for
validation only. The model does not train on it.

1. Start the flight stack.
2. Run the recorder with the label `fault`:

   ```
   python3 scripts/record_telemetry.py --ros-args -p label:=fault
   ```

3. Take off and start the mission.
4. Connect MAVProxy to the same SITL.
5. Weaken motor 0 in the MAVProxy prompt. Set the motor index. Set the thrust
   scale to 60 percent:

   ```
   param set SIM_ENGINE_FAIL 0
   param set SIM_ENGINE_MUL 0.6
   ```

6. Let the mission continue. The autopilot compensates for the weak motor.
7. Stop the recorder with Ctrl-C after the vehicle lands.
8. Restore the motor for the next run. Set the thrust scale back to full:

   ```
   param set SIM_ENGINE_MUL 1.0
   ```

Note: confirm the parameter names against the ArduPilot Copter version in the
image. `SIM_ENGINE_FAIL` selects the motor. `SIM_ENGINE_MUL` scales its
thrust. Older versions can use different names.

Inject the fault early and strong. Set the thrust scale before the vehicle
arms, so the full flight is degraded. A gentle droop of 0.6 is not visible
during a survey. Use 0.5:

```
param set SIM_ENGINE_FAIL 0
param set SIM_ENGINE_MUL 0.5
```

## Train the model

The trainer needs PyTorch. PyTorch is not in the ROS image. Install it one
time in the container as a build tool. onnxruntime is already in the image and
gives the export a parity check.

Do these steps in the container, from `/ws/src/kestrel`:

1. Install PyTorch for the CPU:

   ```
   pip3 install --break-system-packages torch --index-url https://download.pytorch.org/whl/cpu
   ```

2. Run the trainer:

   ```
   python3 scripts/train_health_model.py
   ```

The trainer reads every `telemetry_logs/nominal_*.csv` file. It trains on
nominal data only. It drops any feature that does not vary. It writes two
files:

- `models/health/health_model.onnx`: the trained autoencoder.
- `models/health/health_scaler.json`: the feature list, the scaling values,
  the window size, and the alert threshold.

The trainer then scores every log with onnxruntime. It prints the fraction of
windows above the threshold for each file. A good result shows a low fraction
for the nominal files and a high fraction for the fault file.

Commit both output files. They are small and deterministic.
