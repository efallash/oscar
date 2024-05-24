from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit


def generate_launch_description():
    ld = LaunchDescription()

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "joint_state_broadcaster",
        ],
        output="screen",
    )

    load_l_arm_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "left_arm_controller",
        ],
        output="screen",
    )

    load_r_arm_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "right_arm_controller",
        ],
        output="screen",
    )

    load_l_gripper_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "left_gripper_controller",
        ],
        output="screen",
    )

    load_r_gripper_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "right_gripper_controller",
        ],
        output="screen",
    )

    load_controllers_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_state_broadcaster,
            on_exit=[
                load_l_arm_controller,
                load_r_arm_controller,
                load_l_gripper_controller,
                load_r_gripper_controller,
            ],
        )
    )

    ld.add_action(load_joint_state_broadcaster)
    ld.add_action(load_controllers_event)

    return ld
