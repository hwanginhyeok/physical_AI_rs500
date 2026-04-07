# Current Tasks

> 현재 진행 중인 작업

| # | 태스크 | 시작일 | blocked | 비고 |
|---|--------|--------|---------|------|
| T08 | 실차 배포 — Advantech PC 환경 세팅 | 2026-04-06 | 네트워크 연결 (USB WiFi 동글 or LAN) | HW 확인 완료: Ubuntu 22.04, 8코어, 16GB, 234GB NVMe, CAN0/1, video0~3. ROS2 미설치. Tailscale 있으나 WiFi 안테나 미장착 |
| T07 | 합성 이미지 기반 perception 검증 | 2026-04-04 | Foxglove 실행 확인 | 코드+테스트 완료. `ros2 launch ad_bringup synthetic_test_launch.py` → ws://localhost:8765. [가이드](docs/synthetic_test_guide.md) |
