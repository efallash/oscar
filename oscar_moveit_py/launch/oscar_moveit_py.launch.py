"""
A launch file for running the demo of the moveit_py functionality
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder



def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="oscar", package_name="oscar_moveit_config"
        )
        .robot_description(file_path="config/oscar.urdf.xacro")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .moveit_cpp(
            file_path=get_package_share_directory("oscar_moveit_py")
            + "/config/motion_planning.yaml"
        )
        .to_moveit_configs()
    )

    moveit_config_dict=moveit_config.to_dict()
    moveit_config_dict.update({"use_sim_time":True})
    moveit_py_node = Node(
        name="moveit_py",
        package="oscar_moveit_py",
        executable="oscar_moveit_py",
        output="both",
        parameters=[moveit_config_dict],
    )

    return LaunchDescription([moveit_py_node])