# Debian dependencies extracted by tools/install_local_nav2.py live outside
# the system ROS prefix. Make them visible even for a plain `colcon build`.
get_filename_component(_landerpi_workspace "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
set(_landerpi_dependency_prefix "${_landerpi_workspace}/.deps/nav2_root/opt/ros/humble")
if(EXISTS "${_landerpi_dependency_prefix}/share/nav2_core/cmake/nav2_coreConfig.cmake")
  list(PREPEND CMAKE_PREFIX_PATH "${_landerpi_dependency_prefix}")
  # RMW discovery reads the ament resource index through the environment,
  # independently of CMake's package search path.
  set(ENV{AMENT_PREFIX_PATH} "${_landerpi_dependency_prefix}:$ENV{AMENT_PREFIX_PATH}")
  message(STATUS "LanderPi local dependencies: ${_landerpi_dependency_prefix}")
endif()
