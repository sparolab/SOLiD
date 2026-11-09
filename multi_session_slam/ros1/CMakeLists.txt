cmake_minimum_required(VERSION 3.16)
project(solid_multi_session)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS roscpp nav_msgs sensor_msgs visualization_msgs pcl_conversions)

# SOLiD library (cpp/) with the multi_session component: installed copy or this repository's tree.
find_package(solid 0.2 CONFIG QUIET COMPONENTS slam multi_session)
if(NOT solid_FOUND)
  set(SOLID_USE_PCL OFF CACHE BOOL "" FORCE)
  set(BUILD_SOLID_VERIFY OFF CACHE BOOL "" FORCE)
  set(SOLID_BUILD_MULTI_SESSION ON CACHE BOOL "" FORCE)
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../cpp ${CMAKE_CURRENT_BINARY_DIR}/solid EXCLUDE_FROM_ALL)
endif()

catkin_package()

add_executable(align_node src/align_node.cpp)
target_include_directories(align_node PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(align_node solid::multi_session ${catkin_LIBRARIES})

install(TARGETS align_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch rviz DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
