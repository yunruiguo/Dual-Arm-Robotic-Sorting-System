'''
FilePath: gahi.launch.py
Author: Yunrui Guo
Date: 2024-12-18 16:18:08
LastEditors: Please set LastEditors
LastEditTime: 2024-12-18 16:18:10

Descripttion: 
'''
import launch
from launch import LaunchDescription
from launch.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='your_package_name',
            executable='grasp_task_manager',
            name='grasp_task_manager_node',
            output='screen',
        ),
    ])
