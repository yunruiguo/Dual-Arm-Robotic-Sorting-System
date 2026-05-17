import os

import launch
import launch.actions
import launch.events

import launch_ros
import launch_ros.actions
import launch_ros.events

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():

    grasp_task_manager_param_dir = launch.substitutions.LaunchConfiguration(
        'grasp_task_manager_config_dir',
        default=os.path.join(
            get_package_share_directory('grasp_task_manager'),
            'config',
            'grasp_task_manager.yaml'))
    urdf_param_dir = launch.substitutions.LaunchConfiguration(
        'urdf_param_dir',
        default=os.path.join(
            get_package_share_directory('grasp_task_manager'),
            'urdf',
            'jaka_zu7.urdf'))
    return LaunchDescription([
        Node(
            package='grasp_task_manager',
            executable='grasp_task_manager_node',
            name='grasp_task_manager_node',
            parameters=[{
                    "use_sim_time":False,
                },
                {'robot_description': urdf_param_dir},
                grasp_task_manager_param_dir,
            ], 
            output='screen'
        ),
    ])
