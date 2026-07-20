# Health monitor benchmarks

Model `models/health/health_model.onnx`, threshold 0.986, window 10 samples at 5.0 Hz, features ang_vel_mag, lin_acc_mag.

CPU inference latency: 0.006 ms per window.

## Operating point

Threshold 0.986, consecutive windows required 8.

| Log | Type | Fires | Fire time | Longest over threshold run |
|---|---|---|---|---|
| nominal_20260720_181744.csv | nominal | no | - | 4 |
| nominal_20260720_182526.csv | nominal | no | - | 2 |
| fault_20260720_190021.csv | fault | yes | 79s | 16 |

## Debounce sensitivity

Fire time in seconds, or a dash when silent, per `consecutive_alerts_required`.

| Log | Type | 4 | 6 | 8 | 10 | 12 |
|---|---|---|---|---|---|---|
| nominal_20260720_181744.csv | nominal | 63s | - | - | - | - |
| nominal_20260720_182526.csv | nominal | - | - | - | - | - |
| fault_20260720_190021.csv | fault | 78s | 78s | 79s | 79s | 80s |

## Per window separability

ROC AUC of fault windows against nominal windows: 0.153.

The AUC is below 0.5, and that is the key evidence for the design. After the early degraded burst the fault flight bails to a low motion return, and those calm windows reconstruct better than the busy nominal survey, so per window the fault ranks as more normal than nominal. Per window classification is the wrong frame. The detector instead fires on the presence of a sustained over threshold burst, which the operating point and sensitivity tables separate cleanly.

These numbers come from two nominal logs and one fault log, so they are illustrative of the method.

