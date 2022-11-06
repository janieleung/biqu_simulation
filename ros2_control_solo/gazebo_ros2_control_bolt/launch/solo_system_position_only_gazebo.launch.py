from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import os
import xacro
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    arg_world_filename = PathJoinSubstitution(
        [FindPackageShare("gazebo_ros2_control_bolt"), "world", "bolt_world.world"]
    )

    print("WORLD NODE: DONE")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"])]
        ),
        launch_arguments={
            "verbose": "false",
            "pause": "true",
            "world": arg_world_filename,
        }.items(),
    )

    print("GAZEBO NODE: DONE")


    # Get URDF via xacro
    # Checkpoint 1: URDF change
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("ros2_description_bolt"),
                    "urdf",
                    "test.urdf.xacro",
                ]
            ),
            " use_sim:=true",
        ]
    )
    
    print("URDF CONTENT: DONE")

    robot_description = {"robot_description": robot_description_content}

    print("ROBOT DESCRIPTION: DONE")

    description_path = os.path.join(
        get_package_share_directory('ros2_description_bolt'))

    xacro_file = os.path.join(description_path,
                              'urdf',
                              'solo12_description.xacro')

    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}
    
    
    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[params],
    )

    # Checkpoint 2: Spawn Entity
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "solo", "-x 0", "-y 0", "-z 0"],
        output="screen",
    )

    load_joint_state_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "joint_state_broadcaster",
        ],
        output="screen",
    )

    load_joint_trajectory_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "forward_position_controller",
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=spawn_entity,
                    on_exit=[load_joint_state_controller],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=load_joint_state_controller,
                    on_exit=[load_joint_trajectory_controller],
                )
            ),
            gazebo,
            node_robot_state_publisher,
            spawn_entity,
        ]
    )
