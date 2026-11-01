"""启动 matplotlib 关节监视窗口（订阅 /hand_joint_targets 与 /arm_joint_targets）。

用法:
    ros2 launch ur_inspire_hand hand_arm_monitor.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    monitor = Node(
        package="ur_inspire_hand",
        executable="hand_arm_monitor.py",
        name="hand_arm_monitor",
        output="screen",
    )
    return LaunchDescription([monitor])
