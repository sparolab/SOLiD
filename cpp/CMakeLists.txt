cmake_minimum_required(VERSION 3.16)
project(solid_core LANGUAGES CXX VERSION 0.1.0)

# ---- Modality-neutral SOLiD descriptor core (no ROS) ----
# This is the single C++ source of truth for the SOLiD descriptor; the ros/,
# ros2/ and python/ front-ends all consume it.
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Keep an active Anaconda/Conda install (often first on PATH) from hijacking
# find_package() and injecting its lib dir into RUNPATH — its libstdc++ is older
# than the system toolchain and breaks running against system PCL.
foreach(_conda "$ENV{CONDA_PREFIX}" "$ENV{HOME}/anaconda3" "$ENV{HOME}/miniconda3")
  if(_conda AND EXISTS "${_conda}")
    list(APPEND CMAKE_IGNORE_PREFIX_PATH "${_conda}")
  endif()
endforeach()

find_package(Eigen3 REQUIRED)

# PCL is used only for voxel downsampling and is optional. ON (default) keeps the
# descriptor bit-identical to the original reference (needed for the golden test
# and the ROS wrappers). OFF builds a PCL-free core (internal voxel) for the
# lightweight Python wheel.
option(SOLID_USE_PCL "Use PCL VoxelGrid for downsampling" ON)

add_library(solid_core
  src/extractor.cpp
  src/database.cpp
)
target_include_directories(solid_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(solid_core PUBLIC Eigen3::Eigen)

if(SOLID_USE_PCL)
  # Only common + filters (VoxelGrid). Avoid io/visualization which drag in VTK.
  find_package(PCL REQUIRED COMPONENTS common filters)
  target_include_directories(solid_core PUBLIC ${PCL_INCLUDE_DIRS})
  target_link_libraries(solid_core PUBLIC ${PCL_LIBRARIES})
  target_compile_definitions(solid_core PUBLIC SOLID_USE_PCL)
endif()

add_library(solid::core ALIAS solid_core)

# ---- Verification: solid_core output vs frozen golden descriptors ----
option(BUILD_SOLID_VERIFY "Build the golden regression executable" ON)
if(BUILD_SOLID_VERIFY)
  add_executable(verify_golden test/verify_golden.cpp)
  target_compile_definitions(verify_golden PRIVATE
    SOLID_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/data")
  target_link_libraries(verify_golden PRIVATE solid_core)
endif()

# ---- Optional gtest suite (built only if a system GTest is available) ----
option(BUILD_SOLID_TESTS "Build gtest unit tests if GTest is found" OFF)
if(BUILD_SOLID_TESTS)
  find_package(GTest QUIET)
  if(GTest_FOUND)
    enable_testing()
    add_executable(solid_core_tests test/test_solid_core.cpp)
    target_link_libraries(solid_core_tests PRIVATE solid_core GTest::gtest_main)
    add_test(NAME solid_core_tests COMMAND solid_core_tests)
  else()
    message(STATUS "GTest not found; skipping gtest suite "
                   "(verify_golden still built).")
  endif()
endif()

# ---- Install / export so downstream ros/ ros2/ can find_package(solid_core) ----
include(GNUInstallDirs)
install(TARGETS solid_core EXPORT solid_coreTargets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(FILES third_party/nanoflann.hpp DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(EXPORT solid_coreTargets
  NAMESPACE solid::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/solid_core
)
