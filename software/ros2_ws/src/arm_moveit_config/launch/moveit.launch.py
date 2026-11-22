"""MoveIt 2 for the arm (docs/decisions/0025): move_group and/or RViz with the MotionPlanning panel.

Started by arm_bringup (move_group, next to ros2_control) and by Arm Studio's RViz window:
    ros2 launch arm_moveit_config moveit.launch.py                         # move_group only
    ros2 launch arm_moveit_config moveit.launch.py move_group:=false rviz:=true

Plans are executed by the same arm_controller on every hardware (mock, gazebo, real).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

PLANNING_GROUP = "arm"


def moveit_config(hardware: str):
    """The MoveIt parameters, from the files of this package and the URDF of arm_description."""
    from moveit_configs_utils import MoveItConfigsBuilder

    urdf = f"{FindPackageShare('arm_description').find('arm_description')}/urdf/arm.urdf.xacro"
    return (
        MoveItConfigsBuilder("robotic_arm", package_name="arm_moveit_config")
        .robot_description(file_path=urdf, mappings={"hardware": hardware})
        .robot_description_semantic(file_path="config/arm.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .to_moveit_configs()
    )


def _nodes(context):
    hardware = LaunchConfiguration("hardware").perform(context)
    sim_time = {"use_sim_time": LaunchConfiguration("use_sim_time").perform(context).lower() == "true"}
    config = moveit_config(hardware)
    nodes = []
    if LaunchConfiguration("move_group").perform(context).lower() == "true":
        nodes.append(
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                parameters=[config.to_dict(), sim_time, {"publish_robot_description_semantic": True}],
                output="screen",
            )
        )
    if LaunchConfiguration("rviz").perform(context).lower() == "true":
        rviz = f"{FindPackageShare('arm_moveit_config').find('arm_moveit_config')}/rviz/moveit.rviz"
        nodes.append(
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz],
                parameters=[
                    config.robot_description,
                    config.robot_description_semantic,
                    config.robot_description_kinematics,
                    config.planning_pipelines,
                    config.joint_limits,
                    sim_time,
                ],
                on_exit=Shutdown(),  # closing the window ends the launch, so Arm Studio sees it
            )
        )
    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("hardware", default_value="mock", description="mock | gazebo | real (only names)."),
            DeclareLaunchArgument("use_sim_time", default_value="false", description="true with hardware:=gazebo."),
            DeclareLaunchArgument("move_group", default_value="true", description="Start move_group."),
            DeclareLaunchArgument("rviz", default_value="false", description="Open RViz with MotionPlanning."),
            OpaqueFunction(function=_nodes),
        ]
    )
