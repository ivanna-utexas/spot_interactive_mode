cmake_minimum_required(VERSION 3.8)
project(bva_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/NearbyObstacle.msg"
  "msg/NearbyObstacles.msg"
  "msg/TrajectoryCandidate.msg"
  "msg/TrajectorySet.msg"
  "srv/SelectTrajectory.srv"
  DEPENDENCIES std_msgs nav_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
