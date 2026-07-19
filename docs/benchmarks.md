# Kestrel benchmarks

The script `scripts/score_mission.py` measures these numbers from real
mission runs. The detection rate is the count of defects found, over the
count of ground truth markers. The localization error is the distance
from each detected defect to its nearest ground truth marker. The photos
versus frames streamed value shows how much of the raw camera feed
reaches the report.

The rows for site `pylon` at timestamp `20260718_170647` and for site
`turbine` use the `aruco` detector backend. The row for site `pylon` at
timestamp `20260718_231205` uses the `yolo` detector backend.

| Site | Timestamp | Detection rate | Mean error (m) | Max error (m) | Duration (s) | Photos vs frames streamed |
|---|---|---|---|---|---|---|
| pylon | 20260718_170647 | 4/4 | 2.04 | 2.15 | 349.1 | 4 vs 5236 |
| pylon | 20260718_231205 | 4/4 | 3.21 | 6.72 | 246.2 | 4 vs 3693 |
| turbine | 20260718_183949 | 3/4 | 1.93 | 2.28 | 258.5 | 3 vs 3878 |

The detection rate for the `yolo` row counts the raw number of defects
against the ground truth count. This number does not confirm that each
detection matches a real marker. Only one of the four detections in that
row was a true marker: north 16.31, east -0.01, altitude 4.87, a distance
of 0.37 m from the ground truth position of `marker_0`. The other three
detections are false positives. Each false positive matched to whichever
ground truth marker was nearest.
