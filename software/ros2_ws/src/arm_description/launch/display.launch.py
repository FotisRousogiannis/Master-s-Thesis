"""Visualize the arm model in RViz with sliders for every joint.

Usage:
    ros2 launch arm_description display.launch.py
    ros2 launch arm_description display.launch.py gui:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("arm_description")
    gui = LaunchConfiguration("gui")

    robot_description = ParameterValue(
        Command(["xacro ", PathJoinSubstitution([pkg_share, "urdf", "arm.urdf.xacro"])]),
        value_type=str,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "gui", default_value="true", description="Show joint sliders (joint_state_publisher_gui)."
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"robot_description": robot_description}],
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                condition=IfCondition(gui),
            ),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                condition=UnlessCondition(gui),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", PathJoinSubstitution([pkg_share, "rviz", "display.rviz"])],
            ),
        ]
    )
