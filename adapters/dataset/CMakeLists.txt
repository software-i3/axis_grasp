cmake_minimum_required(VERSION 3.14)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  project(axis_grasp_dataset_project LANGUAGES CXX)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
  get_filename_component(AXIS_GRASP_CPP_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
  add_subdirectory("${AXIS_GRASP_CPP_ROOT}/core"
                   "${CMAKE_CURRENT_BINARY_DIR}/axis_grasp_core")
  add_subdirectory("${AXIS_GRASP_CPP_ROOT}/adapters/common"
                   "${CMAKE_CURRENT_BINARY_DIR}/axis_grasp_common")
endif()

add_library(axis_grasp_dataset_adapter STATIC src/dataset_source.cpp)
target_include_directories(axis_grasp_dataset_adapter PUBLIC include)
target_link_libraries(axis_grasp_dataset_adapter
  PUBLIC axis_grasp_adapter_common axis_grasp_core
)
target_compile_features(axis_grasp_dataset_adapter PUBLIC cxx_std_17)

add_executable(axis_grasp_dataset src/main.cpp)
target_link_libraries(axis_grasp_dataset
  PRIVATE axis_grasp_dataset_adapter axis_grasp_adapter_common axis_grasp_core
)
if(MSVC)
  target_compile_options(axis_grasp_dataset PRIVATE /W4 /permissive-)
else()
  target_compile_options(axis_grasp_dataset PRIVATE -Wall -Wextra -Wpedantic)
endif()
