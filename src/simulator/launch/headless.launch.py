"""Launch a paused headless Gazebo server for controller benchmarks."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions.set_environment_variable import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    """Build the benchmark simulator and ROS-Gazebo bridge launch description."""
    world_path = os.path.join(
        get_package_share_directory('simulator'), 'worlds', 'pendulum.world'
    )
    model_dir = os.path.join(
        get_package_share_directory('simulator'), 'models'
    )
    return LaunchDescription(
        [
            SetEnvironmentVariable(
                name='GZ_SIM_RESOURCE_PATH',
                value=f'/opt/ros/kilted/share:{model_dir}',
            ),
            ExecuteProcess(
                cmd=['gz', 'sim', '-s', '-v', '2', world_path],
                output='screen',
            ),
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=[
                    '--ros-args',
                    '-p',
                    'config_file:=/root/project/src/simulator/bridge.yaml',
                ],
            ),
        ]
    )
