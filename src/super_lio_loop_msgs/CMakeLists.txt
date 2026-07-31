cmake_minimum_required(VERSION 3.16)
project(super_lio_loop_msgs VERSION 0.1.0 LANGUAGES C CXX)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Keyframe.msg"
  "msg/LoopCandidate.msg"
  "srv/GetCorrectedPoses.srv"
  "srv/RunGlobalBA.srv"
  DEPENDENCIES geometry_msgs sensor_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
