cmake_minimum_required(VERSION 3.14)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  project(axis_grasp_core_project LANGUAGES CXX)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
  include(CTest)
  option(AXIS_GRASP_BUILD_TESTS "Build GoogleTest core tests" ON)
  find_package(OpenCV REQUIRED COMPONENTS core imgproc)
endif()

add_library(axis_grasp_core STATIC
  src/axis_voting.cpp
  src/component_filter.cpp
  src/grasp_proposal.cpp
  src/image_ops.cpp
  src/pipeline.cpp
)
target_include_directories(axis_grasp_core
  PUBLIC include
  PRIVATE src ${OpenCV_INCLUDE_DIRS}
)
target_compile_features(axis_grasp_core PUBLIC cxx_std_17)
target_link_libraries(axis_grasp_core PUBLIC ${OpenCV_LIBRARIES})
if(MSVC)
  target_compile_options(axis_grasp_core PRIVATE /W4 /permissive-)
else()
  target_compile_options(axis_grasp_core PRIVATE -Wall -Wextra -Wpedantic)
endif()

if(BUILD_TESTING AND AXIS_GRASP_BUILD_TESTS)
  find_package(GTest QUIET)
  if(GTest_FOUND)
    add_executable(axis_grasp_core_tests
      tests/axis_voting_test.cpp
      tests/component_filter_test.cpp
      tests/grasp_proposal_test.cpp
    )
    target_link_libraries(axis_grasp_core_tests
      PRIVATE axis_grasp_core GTest::gtest_main
    )
    include(GoogleTest)
    gtest_discover_tests(axis_grasp_core_tests)
  else()
    message(WARNING "GoogleTest was not found; core tests are present but will not be built")
  endif()
endif()
