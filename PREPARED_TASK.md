# Prepared Tasks

> Tasks awaiting kickoff

| # | Task | Priority | depends | Notes |
|---|--------|----------|---------|------|
| T01 | LIO-SAM live operation test | P2 | T08 (real-vehicle environment) | Phase 2 |
| T02 | crop_row_detector DL replacement (DeepLabV3) | P2 | Real camera data | Phase 2. Can start after data collection in T08 |
| T04 | Mission manager (automatic orchard patrol) | P2 | T05 ✅ | Approach B: row-end detection → turn → re-align when crop_row is re-detected |
| T06 | CameraIntrinsics simulated camera sync | P3 | — | fx=395, cx=320, cy=240. Revisit when converting Waypoints |

---

## TODO (backlog)

- [ ] Tune crop_row_detector after securing real orchard camera data (Phase 1.5)
- [ ] Replace crop-row recognition with a DL model (DeepLabV3/U-Net) (Phase 2, after securing data)
- [ ] Install and run LIO-SAM for real and test it (WSL2 environment)
- [ ] Re-run scenario simulation + report after resolving matplotlib/numpy compatibility (C48 remainder)
- [ ] Parse HIH-2 xlsx files (product spec sheet, R&D) to reflect accurate vehicle mass/size values
- [ ] Tune physical parameters via System Identification once real-vehicle data is secured
- [ ] Secure YOLO model weight files and verify the inference pipeline
- [ ] CameraIntrinsics simulated camera sync — fx=395, cx=320, cy=240 (T06, before Waypoint conversion)
- [ ] Foxglove visual verification — confirm `/sensor/camera/front` + `/perception/crop_row` via synthetic_test_launch.py (T07)
- [ ] Review terrain traversability classification (Wild Visual Navigation)
- [ ] Build a CI/CD headless simulation pipeline
