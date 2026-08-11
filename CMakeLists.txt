# SPDX-License-Identifier: GPL-3.0
# Copyright (C) 2026 changchuanyong
cmake_minimum_required(VERSION 3.12)

file(READ "package.xml" PACKAGE_XML_CONTENT)
string(REGEX MATCH "<version>([0-9\\.]+)</version>" _ ${PACKAGE_XML_CONTENT})
project(roboparty_head VERSION ${CMAKE_MATCH_1})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
if(NOT CMAKE_CROSSCOMPILING)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")
endif()
find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
  set(CMAKE_CXX_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
endif()

set(THREADS_PREFER_PTHREAD_FLAG ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_BUILD_TYPE Release)

find_package(ament_cmake QUIET)
find_package(roboparty_motors REQUIRED)

add_library(head STATIC
  src/head_controller.cpp
)

target_include_directories(head
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(head PUBLIC roboparty_motors::motors)

install(DIRECTORY include/ DESTINATION include)

install(TARGETS head
  EXPORT roboparty_head-targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

include(CMakePackageConfigHelpers)

configure_package_config_file(
  cmake/roboparty_headConfig.cmake.in
  "${CMAKE_CURRENT_BINARY_DIR}/roboparty_headConfig.cmake"
  INSTALL_DESTINATION lib/cmake/roboparty_head
)

write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/roboparty_headConfigVersion.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)

install(EXPORT roboparty_head-targets
  FILE roboparty_headTargets.cmake
  NAMESPACE roboparty_head::
  DESTINATION lib/cmake/roboparty_head
)

install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/roboparty_headConfig.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/roboparty_headConfigVersion.cmake"
  DESTINATION lib/cmake/roboparty_head
)

if(ament_cmake_FOUND)
  ament_export_targets(roboparty_head-targets HAS_LIBRARY_TARGET)
  ament_export_dependencies(roboparty_motors)
  ament_package()
endif()
