cmake_minimum_required(VERSION 3.8)
project(flask_ui)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(piper_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

install(
  PROGRAMS
    app.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY templates
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
