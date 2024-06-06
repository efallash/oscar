import rclpy
import numpy as np
from math import pi

from rclpy.node import Node
from rclpy.task import Future
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from oscar_interfaces.srv import ArmControl, GripperControl
from gazebo_msgs.srv import GetEntityState, SetEntityState
from geometry_msgs.msg import Pose, Quaternion, Point
from std_msgs.msg import Empty

from tf_transformations import quaternion_from_euler


class ChangeHandsTest(Node):

    def __init__(self):
        super().__init__("change_hands_node")

        # Service Clients
        self.cli_get_state = self.create_client(GetEntityState, "get_entity_state")
        self.cli_set_state = self.create_client(SetEntityState, "set_entity_state")
        self.cli_right_arm = self.create_client(ArmControl, "oscar/right_arm_command")
        self.cli_left_arm = self.create_client(ArmControl, "oscar/left_arm_command")
        self.cli_left_gripper = self.create_client(
            GripperControl, "oscar/left_gripper_command"
        )
        self.cli_right_gripper = self.create_client(
            GripperControl, "oscar/right_gripper_command"
        )

        # Dummy subscription to start the script
        self.dummy_cbg=MutuallyExclusiveCallbackGroup()
        self.create_subscription(Empty, "change_hands_test", self.change_hands, 1, callback_group=self.dummy_cbg)

        self.get_logger().info(
            "Ready to execute test. Publish in /change_hands_test topic to start"
        )

    def call_service(self, client, request) -> Future:
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting again...")
        future = client.call_async(request)
        return future

    async def change_hands(self, _):

        # Object Pick Pose
        obj_orient = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        obj_pose = Pose(position=Point(x=0.4, y=-0.35, z=0.801), orientation=obj_orient)
        # Object move service request
        move_obj = SetEntityState.Request()
        move_obj.state.pose = obj_pose
        move_obj.state.name = "object"
        move_obj.state.reference_frame = "world"

        # Predefined poses for right arm
        pose_pick_right = ArmControl.Request(
            x=0.4, y=-0.35, z=0.8, vel=0.2, named_pose=""
        )
        pose_place_right = ArmControl.Request(
            x=0.25, y=0.0, z=0.82, vel=0.2, named_pose=""
        )
        pose_pre_pick_right = ArmControl.Request(
            x=0.4, y=-0.35, z=0.9, vel=0.2, named_pose=""
        )
        pose_pre_place_right = ArmControl.Request(
            x=0.25, y=0.0, z=0.9, vel=0.2, named_pose=""
        )

        # Predefined poses for left arm
        pose_pick_left = ArmControl.Request(
            x=0.25, y=0.0, z=0.8, vel=0.2, named_pose=""
        )
        pose_place_left = ArmControl.Request(
            x=0.4, y=0.35, z=0.82, vel=0.2, named_pose=""
        )
        pose_pre_pick_left = ArmControl.Request(
            x=0.25, y=0.0, z=0.9, vel=0.2, named_pose=""
        )
        pose_pre_place_left = ArmControl.Request(
            x=0.4, y=0.35, z=0.9, vel=0.2, named_pose=""
        )

        # Named poses
        pose_upright = ArmControl.Request(
            x=0.0, y=0.0, z=0.0, vel=0.2, named_pose="upright"
        )
        pose_home = ArmControl.Request(x=0.0, y=0.0, z=0.0, vel=0.2, named_pose="home")
        close_gripper = GripperControl.Request(close=True)
        open_gripper = GripperControl.Request(close=False)

        # Move Object
        self.get_logger().info("Moving Object")
        move_obj_resp = await self.call_service(self.cli_set_state, move_obj)
        self.get_logger().info(f'Set Entity State: {move_obj_resp.success}')

        # Upright Right
        self.get_logger().info("Going Upright")
        await self.call_service(self.cli_right_arm, pose_upright)

        # Upright Left
        self.get_logger().info("Going Upright")
        await self.call_service(self.cli_left_arm, pose_upright)

        ###RIGHT PICK AND PLACE

        # Gripper
        self.get_logger().info("Right Open Gripper")
        await self.call_service(self.cli_right_gripper, open_gripper)

        # Pre Grasp Position
        self.get_logger().info("Right Pre Pick")
        await self.call_service(self.cli_right_arm, pose_pre_pick_right)

        # Grasp Position
        self.get_logger().info("Right Pick")
        await self.call_service(self.cli_right_arm, pose_pick_right)

        # Gripper
        self.get_logger().info("Right Close Gripper")
        await self.call_service(self.cli_right_gripper, close_gripper)

        # Pre Grasp Position
        self.get_logger().info("Right Pre Pick")
        await self.call_service(self.cli_right_arm, pose_pre_pick_right)

        # Home
        self.get_logger().info("Right Home")
        await self.call_service(self.cli_right_arm, pose_home)

        # Pre Place Position
        self.get_logger().info("Right Pre Place")
        await self.call_service(self.cli_right_arm, pose_pre_place_right)

        # Place Position
        self.get_logger().info("Right Place")
        await self.call_service(self.cli_right_arm, pose_place_right)

        # Gripper
        self.get_logger().info("Right Open Gripper")
        await self.call_service(self.cli_right_gripper, open_gripper)

        # Pre Grasp Position
        self.get_logger().info("Right Pre Place")
        await self.call_service(self.cli_right_arm, pose_pre_place_right)

        # Home
        self.get_logger().info("Right Home")
        await self.call_service(self.cli_right_arm, pose_home)

        ### LEFT PICK AND PLACE

        # Gripper
        self.get_logger().info("Left Open Gripper")
        await self.call_service(self.cli_left_gripper, open_gripper)

        # Pre Grasp Position
        self.get_logger().info("Left Pre Pick")
        await self.call_service(self.cli_left_arm, pose_pre_pick_left)

        # Pre Grasp Position
        self.get_logger().info("Left Pick")
        await self.call_service(self.cli_left_arm, pose_pick_left)

        # Gripper
        self.get_logger().info("Left Close Gripper")
        await self.call_service(self.cli_left_gripper, close_gripper)

        # Pre Grasp Position
        self.get_logger().info("Left Pre Pick")
        await self.call_service(self.cli_left_arm, pose_pre_pick_left)

        # Home
        self.get_logger().info("Left Home")
        await self.call_service(self.cli_left_arm, pose_home)

         # Pre Place Position
        self.get_logger().info("Left Pre Place")
        await self.call_service(self.cli_left_arm, pose_pre_place_left)

        # Place Position
        self.get_logger().info("Left Place")
        await self.call_service(self.cli_left_arm, pose_place_left)

        # Gripper
        self.get_logger().info("Left Open Gripper")
        await self.call_service(self.cli_left_gripper, open_gripper)

        # Pre Grasp Position
        self.get_logger().info("Left Pre Place")
        await self.call_service(self.cli_left_arm, pose_pre_place_left)

        # Home
        self.get_logger().info("Left Home")
        await self.call_service(self.cli_left_arm, pose_home) 

        # SCRIPT END


def main():
    rclpy.init()

    node = ChangeHandsTest()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
