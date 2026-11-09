cmake_minimum_required(VERSION 3.8)
project(landerpi_task_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ExecuteSortTask.action"
  "action/DetectObject.action"
  "action/PickObject.action"
  "action/PlaceObject.action"
  "action/MoveToPose.action"
  "action/MoveToJoint.action"
  "action/MoveNamedPose.action"
  "action/NavigateStation.action"
  "action/FineAlign.action"
  "msg/RobotStatus.msg"
  "srv/ResetFaults.srv"
  DEPENDENCIES geometry_msgs std_msgs builtin_interfaces)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
