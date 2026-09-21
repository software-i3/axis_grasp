cmake_minimum_required(VERSION 3.14)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  project(axis_grasp_adapter_common_project LANGUAGES CXX)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  get_filename_component(AXIS_GRASP_CPP_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
  add_subdirectory("${AXIS_GRASP_CPP_ROOT}/core"
                   "${CMAKE_CURRENT_BINARY_DIR}/axis_grasp_core")
endif()

add_library(axis_grasp_adapter_common STATIC src/adapter_utils.cpp)
target_include_directories(axis_grasp_adapter_common PUBLIC include)
target_link_libraries(axis_grasp_adapter_common PUBLIC axis_grasp_core)
target_compile_features(axis_grasp_adapter_common PUBLIC cxx_std_17)
