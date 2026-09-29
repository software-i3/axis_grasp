# 3.10, not 3.14, so the vehicle's system CMake (3.10.2 on Ubuntu 18.04) can
# configure this directory. Nothing in this file needs a newer policy version;
# see docs/BUILD_AND_RUN.md section 8 for the audit.
cmake_minimum_required(VERSION 3.10)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  project(axis_grasp_adapter_common_project LANGUAGES CXX)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  include(CTest)
  option(AXIS_GRASP_BUILD_TESTS "Build GoogleTest tests" ON)
  get_filename_component(AXIS_GRASP_CPP_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
  add_subdirectory("${AXIS_GRASP_CPP_ROOT}/core"
                   "${CMAKE_CURRENT_BINARY_DIR}/axis_grasp_core")
endif()

add_library(axis_grasp_adapter_common STATIC
  src/adapter_utils.cpp
  src/depth_to_disparity.cpp
  src/polygon_mask.cpp
)
target_include_directories(axis_grasp_adapter_common PUBLIC include)
target_link_libraries(axis_grasp_adapter_common PUBLIC axis_grasp_core)
target_compile_features(axis_grasp_adapter_common PUBLIC cxx_std_17)

# GCC 7 and 8 keep std::filesystem in a separate libstdc++fs archive that has to
# be named on the link line; GCC 9 merged it into libstdc++. The vehicle builds
# with GCC 7.5, where omitting this links nothing and fails with undefined
# references to experimental::filesystem::v1 even though the compile succeeds.
# PUBLIC so the node, dataset and socket executables inherit it through us.
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 9)
  target_link_libraries(axis_grasp_adapter_common PUBLIC stdc++fs)
endif()

if(BUILD_TESTING AND AXIS_GRASP_BUILD_TESTS)
  find_package(GTest QUIET)
  if(GTest_FOUND)
    # CMake below 3.20 names the test main library GTest::Main; the
    # GTest::gtest_main spelling only arrived in 3.20 (and in the config package
    # Ubuntu ships from 20.04 on). Pick whichever exists so the tests link under
    # the vehicle's CMake 3.10.2 as well as a modern CMake.
    if(TARGET GTest::gtest_main)
      set(AXIS_GRASP_GTEST_MAIN GTest::gtest_main)
    elseif(TARGET GTest::Main)
      set(AXIS_GRASP_GTEST_MAIN GTest::Main)
    else()
      set(AXIS_GRASP_GTEST_MAIN ${GTEST_BOTH_LIBRARIES})
    endif()
    add_executable(axis_grasp_polygon_mask_tests tests/polygon_mask_test.cpp)
    target_link_libraries(axis_grasp_polygon_mask_tests
      PRIVATE axis_grasp_adapter_common ${AXIS_GRASP_GTEST_MAIN}
    )
    include(GoogleTest)
    gtest_discover_tests(axis_grasp_polygon_mask_tests)

    add_executable(axis_grasp_depth_to_disparity_tests
      tests/depth_to_disparity_test.cpp)
    target_link_libraries(axis_grasp_depth_to_disparity_tests
      PRIVATE axis_grasp_adapter_common ${AXIS_GRASP_GTEST_MAIN}
    )
    gtest_discover_tests(axis_grasp_depth_to_disparity_tests)
  else()
    message(WARNING "GoogleTest was not found; polygon mask tests are present but will not be built")
  endif()
endif()
