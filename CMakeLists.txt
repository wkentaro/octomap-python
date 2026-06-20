# find_package(Python ... Development.Module) needs CMake 3.18+.
cmake_minimum_required(VERSION 3.18)
project(octomap_python LANGUAGES CXX)

# OctoMap targets an older C++ dialect; pin C++14 so its sources compile
# cleanly (newer default standards make octomap's `byte` typedef clash with
# std::byte).
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(
  Python REQUIRED COMPONENTS Interpreter Development.Module NumPy)

include(FetchContent)

# OctoMap is fetched and pinned to a release tag rather than vendored as a git
# submodule. Bumping the version is a one-line change to GIT_TAG.
#
# Pinned to v1.10.0 to match the package version line (octomap-python 1.10.0.N
# bundles octomap 1.10.0).
FetchContent_Declare(
  octomap
  GIT_REPOSITORY https://github.com/OctoMap/octomap.git
  GIT_TAG v1.10.0
  GIT_SHALLOW TRUE
)
# Populate the sources only; octomap's own CMake build is intentionally not run.
# FetchContent_MakeAvailable() would add_subdirectory(octomap), which both
# defines a CMake target named `octomap` (colliding with python_add_library
# below) and pulls in octovis/Qt. Source-only population needs the low-level
# FetchContent_Populate(), which CMake 3.30 deprecated; CMP0169 keeps that path
# available (the POLICY guard leaves older CMake, which lacks it, untouched).
if(POLICY CMP0169)
  cmake_policy(SET CMP0169 OLD)
endif()
FetchContent_GetProperties(octomap)
if(NOT octomap_POPULATED)
  FetchContent_Populate(octomap)
endif()

set(octomap_dir "${octomap_SOURCE_DIR}")
set(octomap_src "${octomap_dir}/octomap/src")
set(edt_src "${octomap_dir}/dynamicEDT3D/src")

# OctoMap, octomath and dynamicEDT3D are compiled straight into the extension
# so the module is self-contained: no octomap shared libraries to resolve at
# import time. octovis/Qt are simply never referenced here. The source lists
# mirror octomap's own CMakeLists for the pinned release. DynamicEDTOctomap is
# header-only since octomap v1.9.0 (templated over the tree type), so only
# dynamicEDT3D.cpp is compiled; the template is instantiated via octomap.pyx.
set(octomap_sources
  ${octomap_src}/AbstractOcTree.cpp
  ${octomap_src}/AbstractOccupancyOcTree.cpp
  ${octomap_src}/Pointcloud.cpp
  ${octomap_src}/ScanGraph.cpp
  ${octomap_src}/CountingOcTree.cpp
  ${octomap_src}/OcTree.cpp
  ${octomap_src}/OcTreeNode.cpp
  ${octomap_src}/OcTreeStamped.cpp
  ${octomap_src}/ColorOcTree.cpp
  ${octomap_src}/math/Vector3.cpp
  ${octomap_src}/math/Quaternion.cpp
  ${octomap_src}/math/Pose6D.cpp
  ${edt_src}/dynamicEDT3D.cpp
)

add_custom_command(
  OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/octomap.cpp"
  COMMAND Python::Interpreter -m cython --cplus -3
          "${CMAKE_CURRENT_SOURCE_DIR}/octomap/octomap.pyx"
          -o "${CMAKE_CURRENT_BINARY_DIR}/octomap.cpp"
  DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/octomap/octomap.pyx"
          "${CMAKE_CURRENT_SOURCE_DIR}/octomap/octomap_defs.pxd"
          "${CMAKE_CURRENT_SOURCE_DIR}/octomap/dynamicEDT3D_defs.pxd"
  COMMENT "Cythonizing octomap.pyx"
  VERBATIM
)

python_add_library(octomap MODULE
  "${CMAKE_CURRENT_BINARY_DIR}/octomap.cpp"
  ${octomap_sources}
  WITH_SOABI
)
target_include_directories(octomap PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/octomap
  ${octomap_dir}/octomap/include
  ${octomap_dir}/dynamicEDT3D/include
)
target_link_libraries(octomap PRIVATE Python::NumPy)

install(TARGETS octomap DESTINATION .)
