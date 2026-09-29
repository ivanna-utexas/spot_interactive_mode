cmake_minimum_required(VERSION 3.18)
project(people_detector LANGUAGES C CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/People.msg"
  "msg/PeopleArray.msg"
  "msg/MapPerson.msg"
  "msg/MapPersonArray.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

install(PROGRAMS
  scripts/people_format_adapter_node.py
  scripts/nearby_obstacles_bridge_node.py
  scripts/zed_posearray_to_nearby.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES
  launch/pedestrian_tracking.launch.py
  launch/centerpoint_people.launch.py
  launch/people_format_adapter.launch.py
  launch/nearby_obstacles_bridge.launch.py
  DESTINATION share/${PROJECT_NAME}/launch
)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

# CUDA-CenterPoint is intentionally required for this package: a build without
# CUDA/TensorRT would produce a launchable package that cannot provide its
# canonical pipeline.
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(bva_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(CUDAToolkit REQUIRED)
enable_language(CUDA)

find_package(Protobuf 3.21 CONFIG REQUIRED
  PATHS /opt/protobuf-3.21.12/lib/cmake/protobuf
  NO_DEFAULT_PATH
)

set(CENTERPOINT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../Lidar_AI_Solution/CUDA-CenterPoint")
if(NOT EXISTS "${CENTERPOINT_DIR}/include/centerpoint.h")
  message(FATAL_ERROR
    "CUDA-CenterPoint source is missing. Initialize src/Lidar_AI_Solution with "
    "'git submodule update --init --recursive'.")
endif()

find_path(TENSORRT_INCLUDE_DIR NvInfer.h
  HINTS /usr/include/aarch64-linux-gnu /usr/include/x86_64-linux-gnu /usr/include
  REQUIRED
)
find_library(TENSORRT_LIBRARY nvinfer
  HINTS /usr/lib/aarch64-linux-gnu /usr/lib/x86_64-linux-gnu /usr/lib
  REQUIRED
)

set(SPCONV_LIB_DIR
  "${CENTERPOINT_DIR}/3rdparty/libspconv/lib/${CMAKE_HOST_SYSTEM_PROCESSOR}")
find_library(SPCONV_LIBRARY spconv PATHS "${SPCONV_LIB_DIR}" NO_DEFAULT_PATH REQUIRED)

file(GLOB CENTERPOINT_SOURCES CONFIGURE_DEPENDS
  "${CENTERPOINT_DIR}/src/*.cu"
  "${CENTERPOINT_DIR}/src/*.cpp"
)
set(SPCONV_PARSER_SOURCES
  "${CENTERPOINT_DIR}/3rdparty/libspconv/parser/onnx-parser.cpp"
  "${CENTERPOINT_DIR}/3rdparty/libspconv/onnx/onnx-ml.pb.cpp"
  "${CENTERPOINT_DIR}/3rdparty/libspconv/onnx/onnx-operators-ml.pb.cpp"
)

add_executable(centerpoint_people_node
  src/centerpoint_people_node.cpp
  ${CENTERPOINT_SOURCES}
  ${SPCONV_PARSER_SOURCES}
)
target_compile_features(centerpoint_people_node PUBLIC cxx_std_17)
set_target_properties(centerpoint_people_node PROPERTIES
  CUDA_STANDARD 17
  CUDA_STANDARD_REQUIRED ON
  BUILD_RPATH "${SPCONV_LIB_DIR}"
  INSTALL_RPATH "${SPCONV_LIB_DIR}"
)
if(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
  set_property(TARGET centerpoint_people_node PROPERTY CUDA_ARCHITECTURES 87)
endif()
target_include_directories(centerpoint_people_node PRIVATE
  include
  "${TENSORRT_INCLUDE_DIR}"
  "${CENTERPOINT_DIR}/include"
  "${CENTERPOINT_DIR}/3rdparty/libspconv/include"
  "${CENTERPOINT_DIR}/3rdparty/libspconv/parser"
  "${CENTERPOINT_DIR}/3rdparty/libspconv"
)
ament_target_dependencies(centerpoint_people_node
  rclcpp sensor_msgs geometry_msgs std_msgs tf2 tf2_ros bva_msgs
  visualization_msgs diagnostic_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp
)
target_link_libraries(centerpoint_people_node
  "${cpp_typesupport_target}"
  CUDA::cudart
  "${TENSORRT_LIBRARY}"
  "${SPCONV_LIBRARY}"
  protobuf::libprotobuf
  z
  pthread
)
install(TARGETS centerpoint_people_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_tracker test/test_tracker.cpp)
  target_include_directories(test_tracker PRIVATE include)
  target_compile_features(test_tracker PUBLIC cxx_std_17)

  ament_add_gtest(test_sweep_accumulator test/test_sweep_accumulator.cpp)
  target_include_directories(test_sweep_accumulator PRIVATE include)
  target_compile_features(test_sweep_accumulator PUBLIC cxx_std_17)

  ament_add_gtest(test_validation_and_transforms
    test/test_validation_and_transforms.cpp)
  target_include_directories(test_validation_and_transforms PRIVATE include)
  target_compile_features(test_validation_and_transforms PUBLIC cxx_std_17)
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
