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
option(AXIS_GRASP_WITH_DETECTIONS
  "Accept bx_msgs/DetectedInstances contours as an ROI mask source" ON)

# One find_package call: a second one replaces catkin_INCLUDE_DIRS rather than
# adding to it, which silently drops the ROS include directories.
set(AXIS_GRASP_CATKIN_COMPONENTS geometry_msgs roscpp sensor_msgs)
if(AXIS_GRASP_WITH_DETECTIONS)
  list(APPEND AXIS_GRASP_CATKIN_COMPONENTS bx_msgs)
endif()
find_package(catkin REQUIRED COMPONENTS ${AXIS_GRASP_CATKIN_COMPONENTS})
find_package(OpenCV REQUIRED COMPONENTS core imgproc)

catkin_package(
  INCLUDE_DIRS include core/include adapters/common/include
  LIBRARIES axis_grasp_core axis_grasp_adapter_common
  CATKIN_DEPENDS ${AXIS_GRASP_CATKIN_COMPONENTS}
  DEPENDS OpenCV
)

include(CTest)
add_subdirectory(core)
add_subdirectory(adapters/common)

add_library(axis_grasp_mask_pairing STATIC src/mask_pairing.cpp)
target_include_directories(axis_grasp_mask_pairing PUBLIC include)
target_compile_features(axis_grasp_mask_pairing PUBLIC cxx_std_17)

add_executable(axis_grasp_node src/node.cpp src/ros_data_source.cpp)
target_include_directories(axis_grasp_node PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(axis_grasp_node PRIVATE axis_grasp_adapter_common axis_grasp_core axis_grasp_mask_pairing ${catkin_LIBRARIES})
add_dependencies(axis_grasp_node ${catkin_EXPORTED_TARGETS})
if(AXIS_GRASP_WITH_DETECTIONS)
  # Ensure the generated bx_msgs headers exist before the node compiles.
  add_dependencies(axis_grasp_node ${bx_msgs_EXPORTED_TARGETS})
  target_compile_definitions(axis_grasp_node PRIVATE AXIS_GRASP_WITH_DETECTIONS=1)
endif()

if(BUILD_TESTING AND AXIS_GRASP_BUILD_TESTS)
  find_package(GTest QUIET)
  if(GTest_FOUND)
    add_executable(axis_grasp_mask_pairing_tests test/mask_pairing_test.cpp)
    target_link_libraries(axis_grasp_mask_pairing_tests
      PRIVATE axis_grasp_mask_pairing GTest::gtest_main
    )
    include(GoogleTest)
    gtest_discover_tests(axis_grasp_mask_pairing_tests)
  else()
    message(WARNING "GoogleTest was not found; mask pairing tests will not be built")
  endif()
endif()

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
