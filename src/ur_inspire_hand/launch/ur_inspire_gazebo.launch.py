#!/usr/bin/env python3
"""Launch the combined UR arm + Inspire RH56 hand in Gazebo (classic, Gazebo 11).

    ros2 launch ur_inspire_hand ur_inspire_gazebo.launch.py
    ros2 launch ur_inspire_hand ur_inspire_gazebo.launch.py ur_type:=ur10 gui:=false
    ros2 launch ur_inspire_hand ur_inspire_gazebo.launch.py use_rviz:=false

What it starts:
  1. robot_state_publisher      - publishes /robot_description and TF
  2. gzserver / gzclient        - Gazebo classic (gzclient only if gui:=true)
  3. spawn_entity               - spawns the combined robot from /robot_description
  4. controller spawners        - joint_state_broadcaster, arm_controller,
                                  hand_controller (after the model is spawned)
  5. mimic_joint_fixer          - republishes joint states with mimic names fixed
  6. initial_pose               - moves the arm to its home pose and opens the hand
  7. rviz2                      - optional RViz visualisation
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare


def generate_launch_description():
    # ------------------------- arguments -------------------------
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
            description="Robot name / Gazebo model name",
        ),
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start the Gazebo GUI (gzclient). Set false on headless machines.",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Also start RViz2 with the bundled configuration",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation time for ROS nodes",
        ),
        DeclareLaunchArgument(
            "world",
            default_value=PathJoinSubstitution(
                [FindPackageShare("gazebo_ros"), "worlds", "empty.world"]
            ),
            description="Gazebo world file",
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
            "run_initial_pose",
            default_value="true",
            description="Send the arm home pose and open the hand after start-up",
        ),
    ]

    ur_type = LaunchConfiguration("ur_type")
    name = LaunchConfiguration("name")
    gui = LaunchConfiguration("gui")
    use_rviz = LaunchConfiguration("use_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    hand_xyz = LaunchConfiguration("hand_xyz")
    hand_rpy = LaunchConfiguration("hand_rpy")

    # ------------------------- robot description -------------------------
    xacro_file = PathJoinSubstitution(
        [FindPackageShare("ur_inspire_hand"), "urdf", "ur_inspire.urdf.xacro"]
    )
    # NOTE: gazebo_ros2_control passes the robot description to rcl as a
    # '--param robot_description:=<urdf>' rule and rcl parses the value as
    # YAML.  XML comments containing ': ' would break that parse, so the
    # description is produced by the make_gazebo_urdf executable
    # (xacro + comment strip).
    # ParameterValue(..., value_type=str) prevents launch_ros from trying to
    # interpret the URDF string as YAML (it contains ':' characters).
    robot_description = {
        "robot_description": ParameterValue(
            Command(
                [
                    PathJoinSubstitution(
                        [
                            FindPackagePrefix("ur_inspire_hand"),
                            "lib",
                            "ur_inspire_hand",
                            "make_gazebo_urdf",
                        ]
                    ),
                    " ",
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
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
        # mimic joints arrive with a "_mimic" suffix from gazebo_ros2_control;
        # the mimic_joint_fixer node strips it, so listen to the fixed topic.
        remappings=[("joint_states", "joint_states_fixed")],
    )

    # ------------------------- Gazebo -------------------------
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("gazebo_ros"), "/launch/gzserver.launch.py"]
        ),
        launch_arguments={"world": LaunchConfiguration("world")}.items(),
    )
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("gazebo_ros"), "/launch/gzclient.launch.py"]
        ),
        condition=IfCondition(gui),
    )

    # ------------------------- spawn the robot -------------------------
    # spawn_entity.py waits up to 30 s for the gzserver /spawn_entity service.
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic",
            "robot_description",
            "-entity",
            name,
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            "0.0",
        ],
        output="screen",
    )

    # ------------------------- controllers -------------------------
    # Start the controllers once the model (and therefore the
    # gazebo_ros2_control plugin / controller_manager) is up.
    load_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_entity,
            on_exit=[
                TimerAction(
                    period=3.0,
                    actions=[
                        Node(
                            package="controller_manager",
                            executable="spawner",
                            arguments=["joint_state_broadcaster"],
                            output="screen",
                        ),
                        Node(
                            package="controller_manager",
                            executable="spawner",
                            arguments=["arm_controller"],
                            output="screen",
                        ),
                        Node(
                            package="controller_manager",
                            executable="spawner",
                            arguments=["hand_controller"],
                            output="screen",
                        ),
                    ],
                )
            ],
        )
    )

    # ------------------------- helper nodes -------------------------
    mimic_joint_fixer = Node(
        package="ur_inspire_hand",
        executable="mimic_joint_fixer",
        name="mimic_joint_fixer",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    initial_pose = Node(
        package="ur_inspire_hand",
        executable="initial_pose",
        name="initial_pose",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(LaunchConfiguration("run_initial_pose")),
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
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(use_rviz),
    )

    nodes = [
        robot_state_publisher,
        gzserver,
        gzclient,
        spawn_entity,
        load_controllers,
        mimic_joint_fixer,
        initial_pose,
        rviz,
    ]

    return LaunchDescription(declared_arguments + nodes)
