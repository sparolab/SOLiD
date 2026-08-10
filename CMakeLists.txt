cmake_minimum_required(VERSION 3.16)
project(solid_python LANGUAGES CXX)

# Root build for the pip package `solid`: compiles the pybind11 extension from
# the in-tree C++ core (cpp/) so `pip install .` and `pip install git+<repo>`
# work (no sibling-directory problem — everything is under this root).
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Keep an active Anaconda/Conda prefix from hijacking find_package / RUNPATH.
foreach(_conda "$ENV{CONDA_PREFIX}" "$ENV{HOME}/anaconda3" "$ENV{HOME}/miniconda3")
  if(_conda AND EXISTS "${_conda}")
    list(APPEND CMAKE_IGNORE_PREFIX_PATH "${_conda}")
  endif()
endforeach()

find_package(Eigen3 REQUIRED)
find_package(pybind11 CONFIG REQUIRED)

set(CORE "${CMAKE_CURRENT_SOURCE_DIR}/cpp")

# PCL-free build (SOLID_USE_PCL not defined) → internal voxel, Eigen-only.
pybind11_add_module(_core
  python/src/bindings.cpp
  ${CORE}/src/extractor.cpp
  ${CORE}/src/database.cpp
)
target_include_directories(_core PRIVATE ${CORE}/include ${CORE}/third_party)
target_link_libraries(_core PRIVATE Eigen3::Eigen)
# Static C++ runtime so the extension imports under any Python (incl. Conda envs
# such as the GS-ICP-SLAM environment).
target_link_options(_core PRIVATE -static-libstdc++ -static-libgcc)

install(TARGETS _core LIBRARY DESTINATION solid)
