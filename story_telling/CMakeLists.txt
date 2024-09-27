cmake_minimum_required(VERSION 3.5)
project(story_telling)


find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/NewMessage.srv"
  "srv/ObtainResponse.srv"
  "srv/SetupStory.srv"
  DEPENDENCIES std_msgs
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  src/story_telling_services.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

