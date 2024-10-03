cmake_minimum_required(VERSION 3.5)
project(narrative_robot)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  src/robot.py
  src/game_library.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

