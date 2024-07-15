import rclpy
import numpy as np
import threading
from math import pi
from tf_transformations import quaternion_from_euler

from rclpy.node import Node
from oscar_moveit_py.oscar_moveit_py import Oscar

from oscar_interfaces.srv import ArmControl, GripperControl
from geometry_msgs.msg import PoseStamped
from moveit.planning import PlanRequestParameters, PlanningComponent


class OscarCommander(Node):
    def __init__(self):
        super().__init__("oscar_command_server")

        self.oscar = Oscar(self, "oscar_moveit_py")

        # Service Servers
        self.srv_left_arm = self.create_service(
            ArmControl, "oscar/left_arm_command", self.left_arm_callback
        )
        self.srv_right_arm = self.create_service(
            ArmControl, "oscar/right_arm_command", self.right_arm_callback
        )
        self.srv_left_gripper = self.create_service(
            GripperControl, "oscar/left_gripper_command", self.left_gripper_callback
        )
        self.srv_right_gripper = self.create_service(
            GripperControl, "oscar/right_gripper_command", self.right_gripper_callback
        )
        self.get_logger().info("Oscar Command Services are now available.")

    def left_arm_callback(
        self, request: ArmControl.Request, response: ArmControl.Response
    ):
        self.get_logger().info(f"Left arm command received (x= {request.x}, y= {request.y}, z= {request.z}, vel= {request.vel}, pose= {request.named_pose})")
        success, status = self.arm_command(request, arm="left")
        response.success = success
        response.status = status
        return response

    def right_arm_callback(
        self, request: ArmControl.Request, response: ArmControl.Response
    ):
        self.get_logger().info(f"Right arm command received (x= {request.x}, y= {request.y}, z= {request.z}, vel= {request.vel}, pose= {request.named_pose})")
        success, status = self.arm_command(request, arm="right")
        response.success = success
        response.status = status
        return response

    async def left_gripper_callback(
        self, request: GripperControl.Request, response: GripperControl.Response
    ):
        self.get_logger().info("Left gripper command received")
        success = await self.gripper_control(request, "left")
        response.success = success
        return response

    async def right_gripper_callback(
        self, request: GripperControl.Request, response: GripperControl.Response
    ):
        self.get_logger().info("Right gripper command received")
        success = await self.gripper_control(request, "right")
        response.success = success
        return response

    def arm_command(self, request: ArmControl.Request, arm):
        # Check if velocity command is in allowed values
        if request.vel > 0 and request.vel <= 1:
            self.get_logger().info("Executing requested command")
            execute = True  # Flag for execution
        else:
            self.get_logger().info("Planning requested command")
            execute = False

        # If a named pose is commanded
        if len(request.named_pose) > 0:
            success, status = self.oscar.arm_go_to_named_pose(
                arm, request.named_pose, request.vel
            )

            if success:
                if execute:
                    self.get_logger().info(f"Executed pose. Status code: {status}")
                else:
                    self.get_logger().info(
                        f"Pose not executed. Planning code: {status}"
                    )
            else:
                self.get_logger().error(f"Pose failed. Error code: {status}")

            return (success, status)

        # Defining target position relative to the arm base
        arm_x = request.x
        if arm == "left":
            arm_y = request.y - 0.25  # Static transform from the left arm and world
        elif arm == "right":
            arm_y = request.y + 0.25  # Static transform from the right arm and world
        # In case of non existant arm_frame
        else:
            self.get_logger().error("Wrong arm frame")
            return (False, "Wrong Arm Selected")

        # Get object azimuth angle respect to the arm
        if request.x == 0:
            yaw = (pi / 2) * np.sign(arm_y)
        else:
            yaw = np.arctan(arm_y / arm_x)

        # List of pitches to plan
        pitch_list = np.flip(np.linspace(0, pi / 2, num=9))

        # Set planning parameters
        plan_params = PlanRequestParameters(self.oscar.oscar, "ompl")
        plan_params.planning_attempts = 1
        plan_params.planning_pipeline = "ompl"
        plan_params.planning_time = 0.1
        plan_comp = self.oscar.arms[arm]
        pose_link = self.oscar.gripper_links[arm]
        assert isinstance(plan_comp, PlanningComponent)

        # Define pose object
        target_pose = PoseStamped()
        target_pose.header.frame_id = "world"
        target_pose.pose.position.x = request.x
        target_pose.pose.position.y = request.y
        target_pose.pose.position.z = request.z

        # Attempt planning for each pitch angle
        for pitch in pitch_list:
            orient = quaternion_from_euler(0, pitch, yaw)
            target_pose.pose.orientation.x = orient[0]
            target_pose.pose.orientation.y = orient[1]
            target_pose.pose.orientation.z = orient[2]
            target_pose.pose.orientation.w = orient[3]

            plan_comp.set_start_state_to_current_state()
            plan_comp.set_goal_state(pose_stamped_msg=target_pose, pose_link=pose_link)

            plan = self.oscar.plan(
                plan_comp, self.oscar.logger, single_plan_parameters=plan_params
            )

            if plan:
                break

        # Execute if plan was successful
        if plan:
            success, status = self.oscar.arm_go_to_pose(arm, target_pose, request.vel)
        else:
            success = False
            status = "PLAN_FAILED"

        return (success, status)

    async def gripper_control(self, request: GripperControl.Request, arm):
        if request.close:
            handle = await self.oscar.close_gripper(arm)
        else:
            handle = await self.oscar.open_gripper(arm)
        result = await handle.get_result_async()
        success = True if result.result.error_code == 0 else False
        return success


def main():
    rclpy.init()

    oscar_commander = OscarCommander()

    try:
        rclpy.spin(oscar_commander)
    except KeyboardInterrupt:
        oscar_commander.destroy_node()
