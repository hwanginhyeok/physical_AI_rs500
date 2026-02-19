cmake_minimum_required(VERSION 3.8)
project(ad_simulation)

find_package(ament_cmake REQUIRED)

# 모델, 월드, 설정, launch 파일 설치
install(DIRECTORY
  models
  worlds
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
