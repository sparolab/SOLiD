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
find_package(yaml-cpp REQUIRED)

# SOLiD core from this repository (Eigen only, no PCL).
set(SOLID_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../..)
set(SOLID_USE_PCL OFF CACHE BOOL "" FORCE)
set(BUILD_SOLID_VERIFY OFF CACHE BOOL "" FORCE)
add_subdirectory(${SOLID_ROOT}/cpp ${CMAKE_CURRENT_BINARY_DIR}/solid_core EXCLUDE_FROM_ALL)

add_message_files(FILES LoopCandidate.msg)
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs geometry_msgs)

add_executable(solid_node src/solid_node.cpp)
target_include_directories(solid_node PRIVATE
  ${SOLID_ROOT}/slam/common/include ${catkin_INCLUDE_DIRS} ${YAML_CPP_INCLUDE_DIR})
target_link_libraries(solid_node solid::core ${catkin_LIBRARIES} yaml-cpp)
add_dependencies(solid_node ${PROJECT_NAME}_generate_messages_cpp)
target_compile_definitions(solid_node PRIVATE
  SOLID_CONFIG_DIR="${SOLID_ROOT}/slam/config")

install(TARGETS solid_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch rviz DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
