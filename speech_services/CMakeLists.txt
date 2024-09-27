cmake_minimum_required(VERSION 3.5)
project(speech_services)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/TtsEleven.srv"
  "srv/TtsGoogle.srv"
  "srv/TtsMicrosoft.srv"
  "srv/Stt.srv"
  DEPENDENCIES std_msgs
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  src/tts_service.py
  src/stt_service.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
