cmake_minimum_required(VERSION 3.16)
project(solid_multi_session)

# Multi-session SLAM (ROS 1): align_node aligns saved sessions into one map with the SOLiD library
# (solid::backend Aligner: SOLiD inter-session search, GICP check, consensus, one pose graph).
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS roscpp nav_msgs sensor_msgs visualization_msgs pcl_conversions)

# The SOLiD library: an installed copy (find_package) or this repository's cpp/.
find_package(solid 2.0 CONFIG QUIET COMPONENTS slam backend)
if(NOT solid_FOUND AND NOT TARGET solid::backend)
  set(SOLID_USE_PCL OFF CACHE BOOL "" FORCE)
  set(BUILD_SOLID_VERIFY OFF CACHE BOOL "" FORCE)
  set(SOLID_BUILD_BACKEND ON CACHE BOOL "" FORCE)
  set(SOLID_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../../cpp ${CMAKE_CURRENT_BINARY_DIR}/solid EXCLUDE_FROM_ALL)
endif()

catkin_package()

add_executable(align_node src/align_node.cpp)
target_include_directories(align_node PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(align_node solid::backend ${catkin_LIBRARIES})

install(TARGETS align_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch rviz DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY ../config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
