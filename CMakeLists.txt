cmake_minimum_required(VERSION 3.5)
project(acoustic_localization)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

install(PROGRAMS
  nodes/interface_ahoi.py
  nodes/state_estimator.py
  nodes/init_sos.py
  nodes/mqtt_ros_adapter.py
  nodes/control.py
  nodes/acoustic_adapter.py
  nodes/measurement_start_timer.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
