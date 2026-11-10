cmake_minimum_required(VERSION 3.16)
project(solid LANGUAGES CXX VERSION 2.0.0)

# SOLiD library. Everything else in this repository (ROS 1 / ROS 2 nodes, the
# Python package, multi-session SLAM) consumes these targets:
#   solid::core           descriptor + database                  (Eigen)
#   solid::slam           keyframe/loop detection, session writer (header-only; + yaml-cpp loader)
#   solid::backend        loop verification + pose graph (single- and multi-session):
#                         GICP + GTSAM (PCL, GTSAM, yaml-cpp)
# Install it and use find_package(solid COMPONENTS slam backend), or
# add_subdirectory() this folder.
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Keep an active Anaconda/Conda install from hijacking find_package() and RUNPATH
# (its old libstdc++ breaks running against system libraries).
foreach(_conda "$ENV{CONDA_PREFIX}" "$ENV{HOME}/anaconda3" "$ENV{HOME}/miniconda3")
  if(_conda AND EXISTS "${_conda}")
    list(APPEND CMAKE_IGNORE_PREFIX_PATH "${_conda}")
  endif()
endforeach()

option(SOLID_USE_PCL "Use PCL VoxelGrid inside solid::core (bit-identical to the golden test)" ON)
option(SOLID_BUILD_BACKEND "Build solid::backend (needs PCL and GTSAM)" OFF)
option(SOLID_BUILD_MULTI_SESSION "Deprecated alias of SOLID_BUILD_BACKEND" OFF)
if(SOLID_BUILD_MULTI_SESSION)
  set(SOLID_BUILD_BACKEND ON)
endif()
option(BUILD_SOLID_VERIFY "Build the golden regression executable" ON)
option(BUILD_SOLID_TESTS "Build gtest unit tests if GTest is found" OFF)

find_package(Eigen3 REQUIRED)
include(GNUInstallDirs)

# ---- solid::core ----
add_library(solid_core src/extractor.cpp src/database.cpp)
target_include_directories(solid_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(solid_core PUBLIC Eigen3::Eigen)
if(SOLID_USE_PCL)
  find_package(PCL REQUIRED COMPONENTS common filters)
  target_include_directories(solid_core PUBLIC ${PCL_INCLUDE_DIRS})
  target_link_libraries(solid_core PUBLIC ${PCL_LIBRARIES})
  target_compile_definitions(solid_core PUBLIC SOLID_USE_PCL)
endif()
add_library(solid::core ALIAS solid_core)
set(SOLID_TARGETS solid_core)

# ---- solid::slam (header-only; config_yaml.hpp needs yaml-cpp) ----
add_library(solid_slam INTERFACE)
target_link_libraries(solid_slam INTERFACE solid_core)
find_package(yaml-cpp QUIET)
if(yaml-cpp_FOUND)
  if(TARGET yaml-cpp::yaml-cpp)
    target_link_libraries(solid_slam INTERFACE yaml-cpp::yaml-cpp)
  else()
    target_link_libraries(solid_slam INTERFACE yaml-cpp)
  endif()
endif()
add_library(solid::slam ALIAS solid_slam)
list(APPEND SOLID_TARGETS solid_slam)

# ---- solid::backend ----
if(SOLID_BUILD_BACKEND)
  # Some PCL builds pull in a VTK whose targets reference MPI::MPI_C; define it first if present.
  include(CheckLanguage)
  check_language(C)
  if(CMAKE_C_COMPILER)
    enable_language(C)
    find_package(MPI QUIET COMPONENTS C)
  endif()
  find_package(PCL REQUIRED COMPONENTS common filters registration)
  find_package(GTSAM REQUIRED)
  if(NOT yaml-cpp_FOUND)
    message(FATAL_ERROR "solid::backend needs yaml-cpp")
  endif()
  add_library(solid_backend src/backend/session.cpp src/backend/aligner.cpp src/backend/loop_closer.cpp)
  target_include_directories(solid_backend PUBLIC ${PCL_INCLUDE_DIRS})
  target_link_libraries(solid_backend PUBLIC solid_slam gtsam ${PCL_LIBRARIES})
  add_library(solid::backend ALIAS solid_backend)
  list(APPEND SOLID_TARGETS solid_backend)

  # ROS-free tools (like sparo-fast-lio's examples/): loop closing and multi-session alignment
  # on saved sessions, so macOS gets the whole back-end too.
  option(SOLID_BUILD_EXAMPLES "Build the ROS-free back-end tools" ON)
  if(SOLID_BUILD_EXAMPLES)
    add_executable(solid_close_loops examples/close_loops.cpp)
    target_link_libraries(solid_close_loops PRIVATE solid_backend)
    add_executable(solid_align_sessions examples/align_sessions.cpp)
    target_link_libraries(solid_align_sessions PRIVATE solid_backend)
    install(TARGETS solid_close_loops solid_align_sessions RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
  endif()
endif()

# ---- tests ----
if(BUILD_SOLID_VERIFY AND SOLID_USE_PCL)  # reads PCD files with PCL
  add_executable(verify_golden test/verify_golden.cpp)
  target_compile_definitions(verify_golden PRIVATE SOLID_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/data")
  target_link_libraries(verify_golden PRIVATE solid_core)
endif()
if(BUILD_SOLID_TESTS)
  find_package(GTest QUIET)
  if(GTest_FOUND)
    enable_testing()
    add_executable(solid_core_tests test/test_solid_core.cpp)
    target_link_libraries(solid_core_tests PRIVATE solid_core GTest::gtest_main)
    add_test(NAME solid_core_tests COMMAND solid_core_tests)
  else()
    message(STATUS "GTest not found; skipping gtest suite (verify_golden still built).")
  endif()
endif()

# ---- install / export: find_package(solid COMPONENTS slam backend) ----
foreach(_t ${SOLID_TARGETS})
  string(REPLACE "solid_" "" _export_name ${_t})
  set_target_properties(${_t} PROPERTIES EXPORT_NAME ${_export_name})
endforeach()
install(TARGETS ${SOLID_TARGETS} EXPORT solidTargets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(FILES third_party/nanoflann.hpp DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(EXPORT solidTargets NAMESPACE solid:: DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/solid)
include(CMakePackageConfigHelpers)
configure_package_config_file(cmake/solidConfig.cmake.in ${CMAKE_CURRENT_BINARY_DIR}/solidConfig.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/solid)
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/solidConfigVersion.cmake
  COMPATIBILITY SameMajorVersion)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/solidConfig.cmake ${CMAKE_CURRENT_BINARY_DIR}/solidConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/solid)
