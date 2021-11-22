import os

import launch
from launch.actions import GroupAction
import launch_ros.actions
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    gripper = launch_ros.actions.Node(
                package='gripper_simulator',
                executable='gripper_simulator',
                output='screen',
                )

    opcua_bridge = launch_ros.actions.Node(
                package='opcua_ros2_bridge',
                executable='opcua_ros2_bridge',
                output='screen',
                )

    nodes = [
            gripper,
            opcua_bridge,
            ]

    return launch.LaunchDescription(nodes)
