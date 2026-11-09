"""Open only the RViz window, next to a stack that is already running (Arm Studio opens and closes it)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false", description="true with hardware:=gazebo."),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", PathJoinSubstitution([FindPackageShare("arm_description"), "rviz", "display.rviz"])],
                parameters=[{"use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)}],
                on_exit=Shutdown(),  # closing the window ends the launch, so the application sees it
            ),
        ]
    )
