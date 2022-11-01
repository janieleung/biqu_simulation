import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

import xacro
import yaml


def generate_launch_description():
    solo12_description_path = os.path.join(get_package_share_directory('solo12_description'))
    solo12_urdf_path = os.path.join(solo12_description_path,'urdf','solo12.urdf')

    spawn_object = Node(package='gazebo_ros', executable='spawn_entity.py',
                       arguments=['-file', solo12_urdf_path,
                                  '-entity', 'solo',
                                  '-x', '2.0'],
                       output='screen')
    nodes = [
        spawn_object
    ]

    ## Not yet initialize Gazebo nodes

    return LaunchDescription(nodes)
    