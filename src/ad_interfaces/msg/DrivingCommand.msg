# 주행 제어 명령 메시지
std_msgs/Header header

# 목표 속도 (m/s)
float64 target_speed

# 목표 조향각 (rad, 좌회전 양수)
float64 target_steering

# 브레이크 (0.0 ~ 1.0)
float64 brake

# 주행 모드
# 0=IDLE, 1=LANE_KEEPING, 2=LANE_CHANGE,
# 3=OBSTACLE_AVOIDANCE, 4=EMERGENCY_STOP
uint8 driving_mode

# 비상 정지 요청
bool emergency_stop
