cmake_minimum_required(VERSION 3.16)
project(solid_ros)

# Thin ROS 1 wrappers around the SOLiD library (cpp/):
#   solid_node : keyframes, SOLiD loop candidates, online loop closing (solid::slam + solid::backend)
#   align_node : multi-session alignment of saved sessions (solid::backend)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp std_msgs geometry_msgs nav_msgs sensor_msgs visualization_msgs
  message_filters message_generation pcl_conversions)

# The SOLiD library: an installed copy (find_package) or this repository's cpp/.
find_package(solid 0.2 CONFIG QUIET COMPONENTS slam backend)
if(NOT solid_FOUND)
  set(SOLID_USE_PCL OFF CACHE BOOL "" FORCE)
  set(BUILD_SOLID_VERIFY OFF CACHE BOOL "" FORCE)
  set(SOLID_BUILD_BACKEND ON CACHE BOOL "" FORCE)
  set(SOLID_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../../cpp ${CMAKE_CURRENT_BINARY_DIR}/solid EXCLUDE_FROM_ALL)
endif()
set(SOLID_CONFIG_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../config)

add_message_files(FILES LoopCandidate.msg)
generate_messages(DEPENDENCIES std_msgs geometry_msgs)
catkin_package(CATKIN_DEPENDS message_runtime std_msgs geometry_msgs)

add_executable(solid_node src/solid_node.cpp)
target_include_directories(solid_node PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(solid_node solid::backend ${catkin_LIBRARIES})
add_dependencies(solid_node ${PROJECT_NAME}_generate_messages_cpp)
target_compile_definitions(solid_node PRIVATE SOLID_CONFIG_DIR="${SOLID_CONFIG_DIR}")

add_executable(align_node src/align_node.cpp)
target_include_directories(align_node PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(align_node solid::backend ${catkin_LIBRARIES})

install(TARGETS solid_node align_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch rviz DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
