import os

import launch
import launch.actions
import launch.events

import launch_ros
import launch_ros.actions
import launch_ros.events

from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
import lifecycle_msgs.msg
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    ld = launch.LaunchDescription()

    jaka_controller_param_dir = launch.substitutions.LaunchConfiguration(
        'jaka_controller_param_dir',
        default=os.path.join(
            get_package_share_directory('jaka_controller'),
            'params',
            'jaka_controller.yaml'))
    

    jaka_controller_node = Node(
        namespace='jaka_controller',
        package='jaka_controller',
        executable='jaka_controller_node',
        output='screen',
        parameters=[jaka_controller_param_dir]
    )

    ld.add_action(jaka_controller_node)
    return ld
