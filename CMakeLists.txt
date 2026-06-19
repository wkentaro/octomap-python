# find_package(Python ... Development.Module) needs CMake 3.18+.
cmake_minimum_required(VERSION 3.18)
project(octomap_python LANGUAGES CXX)

# OctoMap 1.8.0 predates C++17; on newer compilers the default gnu++17 makes
# octomap's `byte` typedef clash with std::byte.
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(
  Python REQUIRED COMPONENTS Interpreter Development.Module NumPy)

set(octomap_dir "${CMAKE_CURRENT_SOURCE_DIR}/src/octomap")
set(octomap_src "${octomap_dir}/octomap/src")
set(edt_src "${octomap_dir}/dynamicEDT3D/src")

# OctoMap, octomath and dynamicEDT3D are compiled straight into the extension
# so the module is self-contained: no octomap shared libraries to resolve at
# import time. octovis/Qt are simply never referenced here. The source lists
# mirror octomap's own CMakeLists for the pinned v1.8.0 submodule.
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
  ${edt_src}/dynamicEDTOctomap.cpp
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
