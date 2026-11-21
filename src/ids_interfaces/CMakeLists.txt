# =============================================================================
#  ids_interfaces/CMakeLists.txt - generates the project's ROS messages
# -----------------------------------------------------------------------------
#  Turns the four .msg files into C++ and Python message types. Every other package
#  (controller, IDS, planner, attacker, supervisor) talks through these types.
# =============================================================================

cmake_minimum_required(VERSION 3.16)
project(ids_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/JointReference.msg"
  "msg/JointMeasurement.msg"
  "msg/IdsStatus.msg"
  "msg/AttackTruth.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
