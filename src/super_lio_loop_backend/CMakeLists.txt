cmake_minimum_required(VERSION 3.16)
project(super_lio_loop_backend VERSION 0.1.0 LANGUAGES CXX)

if(POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(GTSAM REQUIRED)
find_package(PCL REQUIRED COMPONENTS common kdtree)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(super_lio_loop_msgs REQUIRED)

add_executable(loop_backend_node
  src/BTC.cpp
  src/loop_backend_node.cpp
)
target_include_directories(loop_backend_node PRIVATE
  include
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(loop_backend_node
  Eigen3::Eigen
  ${PCL_LIBRARIES}
)
ament_target_dependencies(loop_backend_node
  pcl_conversions
  rclcpp
  sensor_msgs
  super_lio_loop_msgs
)
if(TARGET gtsam)
  target_link_libraries(loop_backend_node gtsam)
elseif(TARGET GTSAM::gtsam)
  target_link_libraries(loop_backend_node GTSAM::gtsam)
else()
  target_include_directories(loop_backend_node PRIVATE ${GTSAM_INCLUDE_DIR})
  target_link_libraries(loop_backend_node ${GTSAM_LIBRARIES})
endif()

install(TARGETS loop_backend_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)
install(FILES LICENSE README.md THIRD_PARTY_NOTICE.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
