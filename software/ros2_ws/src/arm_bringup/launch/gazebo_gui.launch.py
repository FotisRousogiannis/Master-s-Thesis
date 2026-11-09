"""Open only the Gazebo window, connected to a simulation that already runs (bringup with gui:=false).

Closing the window does not stop the simulation: the server keeps running in the bringup launch.
"""

from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, ExecuteProcess, Shutdown
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            # The window loads the meshes itself (package://arm_description/...).
            AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", [FindPackageShare("arm_description"), "/.."]),
            ExecuteProcess(cmd=["gz", "sim", "-g", "-v", "2"], output="screen", on_exit=Shutdown()),
        ]
    )
