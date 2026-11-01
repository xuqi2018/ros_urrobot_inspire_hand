#!/usr/bin/env python3
"""Visualise the combined UR arm + Inspire RH56 hand in RViz only (no Gazebo).

    ros2 launch ur_inspire_hand ur_inspire_view.launch.py
    ros2 launch ur_inspire_hand ur_inspire_view.launch.py ur_type:=ur10

Joints can be moved with the joint_state_publisher_gui window.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur5",
            description="UR arm type: ur3, ur5, ur10, ur3e, ur5e, ur7e, ur10e, "
            "ur12e, ur16e, ur20, ur30",
        ),
        DeclareLaunchArgument(
            "name",
            default_value="ur_inspire_hand",
            description="Robot name",
        ),
        DeclareLaunchArgument(
            "hand_xyz",
            default_value="-0.001867 -0.000596 0.034255",
            description="Inspire hand mount offset in the UR tool0 frame [m]",
        ),
        DeclareLaunchArgument(
            "hand_rpy",
            default_value="1.5707963267948966 0 3.141592653589793",
            description="Inspire hand mount rotation (roll pitch yaw) in the UR tool0 frame",
        ),
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start joint_state_publisher_gui to move the joints",
        ),
    ]

    ur_type = LaunchConfiguration("ur_type")
    name = LaunchConfiguration("name")
    hand_xyz = LaunchConfiguration("hand_xyz")
    hand_rpy = LaunchConfiguration("hand_rpy")

    xacro_file = PathJoinSubstitution(
        [FindPackageShare("ur_inspire_hand"), "urdf", "ur_inspire.urdf.xacro"]
    )
    # ParameterValue(..., value_type=str) prevents launch_ros from trying to
    # interpret the URDF string as YAML (it contains ':' characters).
    robot_description = {
        "robot_description": ParameterValue(
            Command(
                [
                    "xacro ",
                    xacro_file,
                    " ur_type:=",
                    ur_type,
                    " name:=",
                    name,
                    " hand_xyz:=\"",
                    hand_xyz,
                    "\" hand_rpy:=\"",
                    hand_rpy,
                    "\"",
                ]
            ),
            value_type=str,
        )
    }

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [FindPackageShare("ur_inspire_hand"), "rviz", "ur_inspire.rviz"]
            ),
        ],
    )

    return LaunchDescription(
        declared_arguments + [joint_state_publisher_gui, robot_state_publisher, rviz]
    )
