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

# SIL 테스트 스크립트 설치
install(DIRECTORY
  test
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

# pytest 기반 테스트 탐색 (ament_cmake_pytest 사용 가능 시)
if(BUILD_TESTING)
  find_package(ament_cmake_pytest QUIET)
  if(ament_cmake_pytest_FOUND)
    ament_add_pytest_test(sil_straight_drive
      test/test_straight_drive.py
      TIMEOUT 60
    )
    ament_add_pytest_test(sil_turn
      test/test_turn.py
      TIMEOUT 60
    )
  endif()
endif()

ament_package()
