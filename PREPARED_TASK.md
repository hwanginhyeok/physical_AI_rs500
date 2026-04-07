# Prepared Tasks

> 착수 대기 중인 작업

| # | 태스크 | 우선순위 | depends | 비고 |
|---|--------|----------|---------|------|
| T01 | LIO-SAM 실제 구동 테스트 | P2 | T08 (실차 환경) | Phase 2 |
| T02 | crop_row_detector DL 교체 (DeepLabV3) | P2 | 실제 카메라 데이터 | Phase 2. T08에서 데이터 수집 후 착수 가능 |
| T04 | 미션 매니저 (과수원 자동 순회) | P2 | T05 ✅ | Approach B: 행 끝 감지 → 선회 → crop_row 재감지 시 정렬 |
| T06 | CameraIntrinsics 시뮬 카메라 동기화 | P3 | — | fx=395, cx=320, cy=240. Waypoint 변환 시 재검토 |

---

## TODO (백로그)

- [ ] 과수원 실제 카메라 데이터 확보 후 crop_row_detector 튜닝 (Phase 1.5)
- [ ] DL 모델 (DeepLabV3/U-Net) 기반 작물 행 인식 교체 (Phase 2, 데이터 확보 후)
- [ ] LIO-SAM 실제 설치 및 실행 테스트 (WSL2 환경)
- [ ] matplotlib/numpy 호환 해결 후 시나리오 시뮬레이션 + 리포트 재실행 (C48 잔여)
- [ ] HIH-2 xlsx 파일 (제품규격서, R&D) 파싱하여 차량 질량/크기 정확값 반영
- [ ] 실차 데이터 확보 시 System Identification으로 물리 파라미터 튜닝
- [ ] YOLO 모델 가중치 파일 확보 및 추론 파이프라인 검증
- [ ] CameraIntrinsics 시뮬 카메라 동기화 — fx=395, cx=320, cy=240 (T06, Waypoint 변환 전)
- [ ] Foxglove 시각 검증 — synthetic_test_launch.py로 `/sensor/camera/front` + `/perception/crop_row` 확인 (T07)
- [ ] 지형 traversability 분류 (Wild Visual Navigation) 검토
- [ ] CI/CD headless 시뮬레이션 파이프라인 구축
