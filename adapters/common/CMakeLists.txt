cmake_minimum_required(VERSION 3.14)

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
  src/polygon_mask.cpp
)
target_include_directories(axis_grasp_adapter_common PUBLIC include)
target_link_libraries(axis_grasp_adapter_common PUBLIC axis_grasp_core)
target_compile_features(axis_grasp_adapter_common PUBLIC cxx_std_17)

if(BUILD_TESTING AND AXIS_GRASP_BUILD_TESTS)
  find_package(GTest QUIET)
  if(GTest_FOUND)
    add_executable(axis_grasp_polygon_mask_tests tests/polygon_mask_test.cpp)
    target_link_libraries(axis_grasp_polygon_mask_tests
      PRIVATE axis_grasp_adapter_common GTest::gtest_main
    )
    include(GoogleTest)
    gtest_discover_tests(axis_grasp_polygon_mask_tests)
  else()
    message(WARNING "GoogleTest was not found; polygon mask tests are present but will not be built")
  endif()
endif()
