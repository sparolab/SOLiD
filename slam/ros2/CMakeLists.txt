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
find_package(yaml-cpp REQUIRED)

# SOLiD core from this repository (Eigen only, no PCL).
set(SOLID_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../..)
set(SOLID_USE_PCL OFF CACHE BOOL "" FORCE)
set(BUILD_SOLID_VERIFY OFF CACHE BOOL "" FORCE)
add_subdirectory(${SOLID_ROOT}/cpp ${CMAKE_CURRENT_BINARY_DIR}/solid_core EXCLUDE_FROM_ALL)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/LoopCandidate.msg
  DEPENDENCIES std_msgs builtin_interfaces geometry_msgs)
rosidl_get_typesupport_target(SOLID_MSGS ${PROJECT_NAME} rosidl_typesupport_cpp)

add_executable(solid_node src/solid_node.cpp)
target_include_directories(solid_node PRIVATE ${SOLID_ROOT}/slam/common/include)
target_link_libraries(solid_node solid::core yaml-cpp ${SOLID_MSGS})
ament_target_dependencies(solid_node
  rclcpp std_msgs geometry_msgs nav_msgs sensor_msgs visualization_msgs message_filters)

install(TARGETS solid_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME})
install(DIRECTORY ${SOLID_ROOT}/slam/config DESTINATION share/${PROJECT_NAME})

ament_package()
