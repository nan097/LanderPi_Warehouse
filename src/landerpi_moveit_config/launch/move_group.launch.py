from launch import LaunchDescription
from launch.actions import LogInfo


def generate_launch_description():
    return LaunchDescription([LogInfo(msg='NOT_IMPLEMENTED: confirm URDF/SRDF, calibrated joint limits and controllers before enabling MoveIt')])
