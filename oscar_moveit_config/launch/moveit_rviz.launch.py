from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_moveit_rviz_launch


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("oscar", package_name="oscar_moveit_config")
        .robot_description(file_path="config/oscar.urdf.xacro")
        .robot_description_kinematics()
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(
            pipelines=["ompl"],
            default_planning_pipeline="ompl",
        )
        .to_moveit_configs()
    )
    return generate_moveit_rviz_launch(moveit_config)
