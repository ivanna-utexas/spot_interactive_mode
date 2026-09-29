cmake_minimum_required(VERSION 3.8)
project(tl_expected)

find_package(ament_cmake REQUIRED)

add_library(tl_expected INTERFACE)
target_include_directories(tl_expected INTERFACE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS tl_expected
  EXPORT export_tl_expected
)

ament_export_targets(export_tl_expected)
ament_package()
