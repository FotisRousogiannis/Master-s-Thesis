"""Start the arm's control stack on the selected hardware (docs/decisions/0016).

Usage:
    ros2 launch arm_bringup bringup.launch.py                      # mock hardware + RViz
    ros2 launch arm_bringup bringup.launch.py hardware:=gazebo     # Gazebo Harmonic + RViz
    ros2 launch arm_bringup bringup.launch.py rviz:=false

Everything above ros2_control (controllers, GUI, MoveIt) is identical for every hardware.
"""

from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

SUPPORTED_HARDWARE = ("mock", "gazebo")  # "real" follows with arm_hardware + micro-ROS Agent
CONTROLLERS = ("joint_state_broadcaster", "arm_controller")


def _stack(context):
    hardware = LaunchConfiguration("hardware").perform(context)
    if hardware not in SUPPORTED_HARDWARE:
        raise RuntimeError(
            f"hardware:={hardware} is not supported yet; choose one of {', '.join(SUPPORTED_HARDWARE)}"
        )
    sim = hardware == "gazebo"

    description_share = FindPackageShare("arm_description").perform(context)
    bringup_share = FindPackageShare("arm_bringup").perform(context)
    controllers_file = f"{bringup_share}/config/controllers.yaml"

    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                f"{description_share}/urdf/arm.urdf.xacro",
                f" hardware:={hardware}",
                f" controllers_file:={controllers_file}",
            ]
        ),
        value_type=str,
    )

    actions = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description, "use_sim_time": sim}],
            output="screen",
        ),
    ]

    if sim:
        # Gazebo loads controller_manager itself (gz_ros2_control plugin in the URDF).
        actions += [
            # Let Gazebo resolve package://arm_description/... meshes.
            AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", f"{description_share}/.."),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [FindPackageShare("ros_gz_sim").perform(context), "/launch/gz_sim.launch.py"]
                ),
                launch_arguments={"gz_args": f"-r -v 2 {bringup_share}/worlds/arm_world.sdf"}.items(),
            ),
            Node(
                package="ros_gz_sim",
                executable="create",
                arguments=["-topic", "robot_description", "-name", "robotic_arm"],
                output="screen",
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
                output="screen",
            ),
        ]
    else:
        actions.append(
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[controllers_file],
                # controller_manager reads the URDF from the topic published by robot_state_publisher.
                remappings=[("~/robot_description", "/robot_description")],
                output="screen",
            )
        )

    actions += [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[name, "--controller-manager", "/controller_manager", "--controller-manager-timeout", "60"],
            parameters=[{"use_sim_time": sim}],
            output="screen",
        )
        for name in CONTROLLERS
    ]

    actions.append(
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", f"{description_share}/rviz/display.rviz"],
            parameters=[{"use_sim_time": sim}],
            condition=IfCondition(LaunchConfiguration("rviz")),
        )
    )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "hardware",
                default_value="mock",
                description="Hardware behind ros2_control: mock | gazebo (real: not implemented yet).",
            ),
            DeclareLaunchArgument("rviz", default_value="true", description="Open RViz."),
            OpaqueFunction(function=_stack),
        ]
    )
