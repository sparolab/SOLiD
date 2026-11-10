cmake_minimum_required(VERSION 3.16)
project(solid_multi_session)

# Multi-session SLAM (ROS 2): align_node aligns saved sessions into one map with the SOLiD library.
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)

# The SOLiD library: an installed copy (find_package) or this repository's cpp/.
find_package(solid 2.0 CONFIG QUIET COMPONENTS slam backend)
if(NOT solid_FOUND AND NOT TARGET solid::backend)
  set(SOLID_USE_PCL OFF CACHE BOOL "" FORCE)
  set(BUILD_SOLID_VERIFY OFF CACHE BOOL "" FORCE)
  set(SOLID_BUILD_BACKEND ON CACHE BOOL "" FORCE)
  set(SOLID_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../../cpp ${CMAKE_CURRENT_BINARY_DIR}/solid EXCLUDE_FROM_ALL)
endif()

add_executable(align_node src/align_node.cpp)
target_link_libraries(align_node solid::backend)
ament_target_dependencies(align_node rclcpp sensor_msgs visualization_msgs pcl_conversions)

install(TARGETS align_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME})
install(DIRECTORY ../config DESTINATION share/${PROJECT_NAME})

ament_package()
