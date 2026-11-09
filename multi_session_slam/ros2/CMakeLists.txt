cmake_minimum_required(VERSION 3.16)
project(solid_multi_session)

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

# SOLiD library (cpp/) with the multi_session component: installed copy or this repository's tree.
find_package(solid 0.2 CONFIG QUIET COMPONENTS slam multi_session)
if(NOT solid_FOUND)
  set(SOLID_USE_PCL OFF CACHE BOOL "" FORCE)
  set(BUILD_SOLID_VERIFY OFF CACHE BOOL "" FORCE)
  set(SOLID_BUILD_MULTI_SESSION ON CACHE BOOL "" FORCE)
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../cpp ${CMAKE_CURRENT_BINARY_DIR}/solid EXCLUDE_FROM_ALL)
endif()

add_executable(align_node src/align_node.cpp)
target_link_libraries(align_node solid::multi_session)
ament_target_dependencies(align_node rclcpp sensor_msgs visualization_msgs pcl_conversions)

install(TARGETS align_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME})
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../config/ DESTINATION share/${PROJECT_NAME}/config)

ament_package()
