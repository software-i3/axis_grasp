# 3.10, not 3.14, so the vehicle's system CMake (3.10.2 on Ubuntu 18.04) can
# configure this directory. Nothing in this file needs a newer policy version;
# see docs/BUILD_AND_RUN.md section 8 for the audit.
cmake_minimum_required(VERSION 3.10)

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
    # CMake below 3.20 names the test main library GTest::Main; the
    # GTest::gtest_main spelling only arrived in 3.20 (and in the config package
    # Ubuntu ships from 20.04 on). Pick whichever exists so the tests link under
    # the vehicle's CMake 3.10.2 as well as a modern CMake. Without this, a
    # GTest that is found on the vehicle fails the link with
    # "cannot find -lGTest::gtest_main".
    if(TARGET GTest::gtest_main)
      set(AXIS_GRASP_GTEST_MAIN GTest::gtest_main)
    elseif(TARGET GTest::Main)
      set(AXIS_GRASP_GTEST_MAIN GTest::Main)
    else()
      set(AXIS_GRASP_GTEST_MAIN ${GTEST_BOTH_LIBRARIES})
    endif()
    add_executable(axis_grasp_core_tests
      tests/axis_voting_test.cpp
      tests/component_filter_test.cpp
      tests/grasp_proposal_test.cpp
    )
    target_link_libraries(axis_grasp_core_tests
      PRIVATE axis_grasp_core ${AXIS_GRASP_GTEST_MAIN}
    )
    include(GoogleTest)
    gtest_discover_tests(axis_grasp_core_tests)
  else()
    message(WARNING "GoogleTest was not found; core tests are present but will not be built")
  endif()
endif()
