cmake_minimum_required(VERSION 3.16)
project(solid_ros)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pcl_conversions QUIET)  # only align_node needs it

# SOLiD library (cpp/): an installed copy (find_package) or this repository's tree.
find_package(solid 0.2 CONFIG QUIET COMPONENTS slam backend)
if(NOT solid_FOUND)
  set(SOLID_USE_PCL OFF CACHE BOOL "" FORCE)
  set(BUILD_SOLID_VERIFY OFF CACHE BOOL "" FORCE)
  set(SOLID_BUILD_BACKEND ON CACHE BOOL "" FORCE)
  set(SOLID_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../../cpp ${CMAKE_CURRENT_BINARY_DIR}/solid EXCLUDE_FROM_ALL)
endif()
set(SOLID_CONFIG_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../config)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/LoopCandidate.msg
  DEPENDENCIES std_msgs builtin_interfaces geometry_msgs)
rosidl_get_typesupport_target(SOLID_MSGS ${PROJECT_NAME} rosidl_typesupport_cpp)

add_executable(solid_node src/solid_node.cpp)
target_link_libraries(solid_node solid::backend ${SOLID_MSGS})
ament_target_dependencies(solid_node
  rclcpp std_msgs geometry_msgs nav_msgs sensor_msgs visualization_msgs message_filters)

install(TARGETS solid_node DESTINATION lib/${PROJECT_NAME})
if(pcl_conversions_FOUND)
  add_executable(align_node src/align_node.cpp)
  target_link_libraries(align_node solid::backend)
  ament_target_dependencies(align_node rclcpp sensor_msgs visualization_msgs pcl_conversions)
  install(TARGETS align_node DESTINATION lib/${PROJECT_NAME})
endif()
install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME})
install(DIRECTORY ${SOLID_CONFIG_DIR}/ DESTINATION share/${PROJECT_NAME}/config)

ament_package()
