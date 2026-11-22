"""Open only the RViz window, next to a stack that is already running (Arm Studio opens and closes it).

moveit:=true (default): RViz with the MotionPlanning panel of MoveIt (ADR-0025); false: only the model.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, Shutdown
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false", description="true with hardware:=gazebo."),
            DeclareLaunchArgument("moveit", default_value="true", description="MotionPlanning panel (MoveIt)."),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare("arm_moveit_config"), "launch", "moveit.launch.py"])
                ),
                launch_arguments={
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "move_group": "false",
                    "rviz": "true",
                }.items(),
                condition=IfCondition(LaunchConfiguration("moveit")),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", PathJoinSubstitution([FindPackageShare("arm_description"), "rviz", "display.rviz"])],
                parameters=[{"use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)}],
                on_exit=Shutdown(),  # closing the window ends the launch, so the application sees it
                condition=UnlessCondition(LaunchConfiguration("moveit")),
            ),
        ]
    )
