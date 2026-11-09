cmake_minimum_required(VERSION 3.16)
project(solid_ros)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp std_msgs geometry_msgs nav_msgs sensor_msgs visualization_msgs
  message_filters message_generation)
find_package(Eigen3 REQUIRED)

# SOLiD library (cpp/): an installed copy (find_package) or this repository's tree.
find_package(solid 0.2 CONFIG QUIET COMPONENTS slam)
if(NOT solid_FOUND)
  set(SOLID_USE_PCL OFF CACHE BOOL "" FORCE)
  set(BUILD_SOLID_VERIFY OFF CACHE BOOL "" FORCE)
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../cpp ${CMAKE_CURRENT_BINARY_DIR}/solid EXCLUDE_FROM_ALL)
endif()
set(SOLID_CONFIG_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../config)

add_message_files(FILES LoopCandidate.msg)
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs geometry_msgs)

add_executable(solid_node src/solid_node.cpp)
target_include_directories(solid_node PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(solid_node solid::slam ${catkin_LIBRARIES})
add_dependencies(solid_node ${PROJECT_NAME}_generate_messages_cpp)
target_compile_definitions(solid_node PRIVATE
  SOLID_CONFIG_DIR="${SOLID_CONFIG_DIR}")

install(TARGETS solid_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch rviz DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
