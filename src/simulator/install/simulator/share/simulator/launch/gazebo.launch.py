from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

from launch.actions.set_environment_variable import SetEnvironmentVariable


def generate_launch_description():
    world_path = os.path.join(
        get_package_share_directory("simulator"),
        "worlds",
        "pendulum.world",
    )
    model_dir = os.path.join(
        get_package_share_directory("simulator"), "models"
    )
    return LaunchDescription(
        [
            SetEnvironmentVariable(
                name="GZ_SIM_RESOURCE_PATH", value=f"/opt/ros/kilted/share:{model_dir}"
            ),
            ExecuteProcess(
                cmd=[
                    "gz",
                    "sim",
                    "-v",
                    "4",
                    f"{world_path}",
                    "--render-engine",
                    "ogre",
                ],
                output="screen",
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                arguments=[
                    "--ros-args",
                    "-p",
                    "config_file:=/root/project/src/simulator/bridge.yaml",
                ],
            ),
        ]
    )
