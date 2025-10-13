cmake_minimum_required(VERSION 3.8)
project(viseme_renderer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)


install(PROGRAMS
  src/VisemeRenderer.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
