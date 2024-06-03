cmake_minimum_required(VERSION 3.5)
project(acoustic_localization)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

install(PROGRAMS
  nodes/interface_ahoi.py
  nodes/state_estimator.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
