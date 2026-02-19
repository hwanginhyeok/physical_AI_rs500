# 차량 상태 메시지
std_msgs/Header header

# 위치 (GPS 좌표)
float64 latitude
float64 longitude
float64 altitude

# 속도 (m/s)
float64 speed

# 방향 (rad, 북쪽 기준 시계방향)
float64 heading

# 조향각 (rad)
float64 steering_angle

# 가속도 (m/s^2)
float64 acceleration

# 기어 상태: 0=P, 1=R, 2=N, 3=D
uint8 gear

# 차량 상태 플래그
bool engine_on
bool autonomous_mode
