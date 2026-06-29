# Current Tasks

> Tasks currently in progress

| # | Task | Start date | blocked | Notes |
|---|--------|--------|---------|------|
| T08 | Real-vehicle deployment — Advantech PC environment setup | 2026-04-06 | Network connection (USB WiFi dongle or LAN) | HW check done: Ubuntu 22.04, 8 cores, 16GB, 234GB NVMe, CAN0/1, video0~3. ROS2 not installed. Tailscale present but WiFi antenna not attached |
| T07 | Perception validation based on synthetic images | 2026-04-04 | Verify Foxglove execution | Code + tests done. `ros2 launch ad_bringup synthetic_test_launch.py` → ws://localhost:8765. [Guide](docs/synthetic_test_guide.md) |
