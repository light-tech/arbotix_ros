cmake_minimum_required(VERSION 3.23)
project(arbotix_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Analog.msg"
  "msg/Digital.msg"
  "srv/Enable.srv"
  "srv/Relax.srv"
  "srv/SetupChannel.srv"
  "srv/SetSpeed.srv"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
