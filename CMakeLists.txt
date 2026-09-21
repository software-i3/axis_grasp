cmake_minimum_required(VERSION 3.10)
project(axis_grasp VERSION 0.2.0 LANGUAGES CXX)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS Debug Release RelWithDebInfo MinSizeRel)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(AXIS_GRASP_BUILD_DATASET "Build the offline dataset executable" ON)
option(AXIS_GRASP_BUILD_SOCKET "Build the uvgRTP receiver executable" OFF)
option(AXIS_GRASP_BUILD_TESTS "Build core tests when GoogleTest is available" ON)

find_package(catkin REQUIRED COMPONENTS geometry_msgs roscpp sensor_msgs)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)

catkin_package(
  INCLUDE_DIRS include core/include adapters/common/include
  LIBRARIES axis_grasp_core axis_grasp_adapter_common
  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs
  DEPENDS OpenCV
)

include(CTest)
add_subdirectory(core)
add_subdirectory(adapters/common)

add_executable(axis_grasp_node src/node.cpp src/ros_data_source.cpp)
target_include_directories(axis_grasp_node PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(axis_grasp_node PRIVATE axis_grasp_adapter_common axis_grasp_core ${catkin_LIBRARIES})
add_dependencies(axis_grasp_node ${catkin_EXPORTED_TARGETS})

if(AXIS_GRASP_BUILD_DATASET)
  add_subdirectory(adapters/dataset)
endif()

if(AXIS_GRASP_BUILD_SOCKET)
  set(UVGRTP_DISABLE_CRYPTO ON CACHE BOOL "" FORCE)
  set(UVGRTP_DISABLE_TESTS ON CACHE BOOL "" FORCE)
  set(UVGRTP_DISABLE_EXAMPLES ON CACHE BOOL "" FORCE)
  set(UVGRTP_DISABLE_INSTALL ON CACHE BOOL "" FORCE)
  add_subdirectory(third_party/uvgrtp EXCLUDE_FROM_ALL)
  add_subdirectory(adapters/socket)
endif()

install(TARGETS axis_grasp_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/axis_grasp/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY config launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
