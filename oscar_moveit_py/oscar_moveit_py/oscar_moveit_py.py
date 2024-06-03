#!/usr/bin/env python3
"""
Wrapper for using the moveit_py API with the OSCAR Robot.
"""

import time
import threading

# generic ros libraries
import rclpy
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.action import ActionClient

# moveit python library
from moveit.core.robot_state import RobotState
from moveit.core.robot_trajectory import RobotTrajectory
from moveit.planning import (
    MoveItPy,
    PlanningComponent,
    TrajectoryExecutionManager
)


from geometry_msgs.msg import PoseStamped
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from tf_transformations import euler_from_quaternion

class Oscar():
    def __init__(self, node: Node, name='moveit_py'):

        self.logger = get_logger(f'oscar.{name}')
        # instantiate MoveItPy instance and get planning component
        self.oscar = MoveItPy(node_name=name)
        trajectory_execution = self.oscar.get_trajactory_execution_manager()
        assert isinstance(trajectory_execution, TrajectoryExecutionManager)
        trajectory_execution.enable_execution_duration_monitoring(True)
        trajectory_execution.set_allowed_execution_duration_scaling(1.2)

        # Create objects for the arms and grippers
        self.arms = {}
        self.gripper_links = {}
        self.grippers = {}
        self.gripper_goal_msgs = {}

        # Populate arms and grippers #TODO: Add support for controlling both arms in a single group
        self.arms['right'] = self.oscar.get_planning_component("right_arm")
        self.arms['left'] = self.oscar.get_planning_component("left_arm")
        self.gripper_links['right'] = 'right_arm_gripper_link'
        self.gripper_links['left'] = 'left_arm_gripper_link'
        self.grippers['right'] = ActionClient(node, FollowJointTrajectory, '/right_gripper_controller/follow_joint_trajectory')
        self.grippers['left'] = ActionClient(node, FollowJointTrajectory, '/left_gripper_controller/follow_joint_trajectory')
        self.gripper_goal_msgs['right']=FollowJointTrajectory.Goal()
        self.gripper_goal_msgs['left']=FollowJointTrajectory.Goal()
        self.gripper_goal_msgs['right'].trajectory.joint_names=['right_arm_finger1_prismatic', 'right_arm_finger2_prismatic']
        self.gripper_goal_msgs['left'].trajectory.joint_names=['left_arm_finger1_prismatic', 'left_arm_finger2_prismatic']
        self.logger.info("MoveItPy instance created")

    def close_gripper(self, arm, sleep_time=0.1):
        point=JointTrajectoryPoint()
        point.positions=[0.0, 0.0]
        self.logger.info("Closing gripper")
        future=self.gripper_action(arm, point, sleep_time)
        return future

    def open_gripper(self, arm, sleep_time=0.1):
        point=JointTrajectoryPoint()
        point.positions=[0.02, 0.02]
        self.logger.info("Opening gripper")
        future=self.gripper_action(arm, point, sleep_time)
        return future
    
    def gripper_action(self, arm, point:JointTrajectoryPoint, sleep_time):
        if arm=='left' or arm=='right':
            goal_msg=self.gripper_goal_msgs[arm]
            gripper=self.grippers[arm]
        else:
            self.logger.error('Wrong Arm Selected: Arm must be "left" or "right".')
            return False
        assert isinstance(goal_msg, FollowJointTrajectory.Goal)
        assert isinstance(gripper, ActionClient)
        goal_msg.trajectory.points=[point]
        future = gripper.send_goal_async(goal_msg)
        time.sleep(sleep_time)
        return future


    def arm_go_to_named_pose(self, arm: str, pose_name: str, vel_factor=0.2, sleep_time=0.1): #TODO: Add acceleration and velocity scaling to all methods
        if arm=='left' or arm=='right':
            thor_arm=self.arms[arm]
        else:
            self.logger.error('Wrong Arm Selected: Arm must be "left" or "right".')
            return False
        assert isinstance(thor_arm, PlanningComponent)
        thor_arm.set_start_state_to_current_state()
        thor_arm.set_goal_state(configuration_name=pose_name)
        self.logger.info(f"Moving to pose: {pose_name}")
        return self.plan_and_execute(self.oscar, thor_arm, self.logger, vel_factor=vel_factor, sleep_time=sleep_time)

    def arm_go_to_pose(self, arm: str, pose: PoseStamped, vel_factor=0.2, sleep_time=0.1):
        if arm=='left' or arm=='right':
            thor_arm=self.arms[arm]
            gripper_link=self.gripper_links[arm]
        else:
            self.logger.error('Wrong Arm Selected: Arm must be "left" or "right".')
            return False
        assert isinstance(thor_arm, PlanningComponent)
        thor_arm.set_start_state_to_current_state()
        thor_arm.set_goal_state(pose_stamped_msg=pose, pose_link=gripper_link)
        angles=euler_from_quaternion([pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w])
        self.logger.info(f'Moving to pose x: {pose.pose.position.x} y: {pose.pose.position.y} z: {pose.pose.position.z} r: {angles[0]} p: {angles[1]} y: {angles[2]}')
        return self.plan_and_execute(self.oscar, thor_arm, self.logger, vel_factor=vel_factor, sleep_time=sleep_time)


    def plan(self,
        planning_component: PlanningComponent,
        logger,
        single_plan_parameters=None,
        multi_plan_parameters=None
        ):
        """Helper function to plan and execute a motion."""
        # plan to goal
        logger.info("Planning trajectory")
        if multi_plan_parameters is not None:
            plan_result = planning_component.plan(
                multi_plan_parameters=multi_plan_parameters
            )
        elif single_plan_parameters is not None:
            plan_result = planning_component.plan(
                single_plan_parameters=single_plan_parameters
            )
        else:
            plan_result = planning_component.plan()
        return plan_result

    def execute(self,
        robot: MoveItPy,
        logger,
        plan_result,
        vel_factor=1,
        sleep_time=0.0        
        ):
        # execute the plan
        logger.info("Executing plan")
        robot_trajectory = plan_result.trajectory
        assert isinstance(robot_trajectory, RobotTrajectory)
        robot_trajectory.apply_totg_time_parameterization(vel_factor, 1.0)
        result=robot.execute(robot_trajectory, controllers=[])
        time.sleep(sleep_time)
        return result

    def plan_and_execute(self,
        robot,
        planning_component,
        logger,
        single_plan_parameters=None,
        multi_plan_parameters=None,
        vel_factor=1,
        sleep_time=0.0,
    ):

        logger.info('Planning and executing')
        plan_result=self.plan(planning_component,logger,single_plan_parameters,multi_plan_parameters)

        if plan_result:
            if vel_factor>0:
                execute_result=self.execute(robot,logger,plan_result, vel_factor, sleep_time)

                if execute_result.status=='SUCCEEDED':
                    logger.info(f'EXECUTION SUCCEEDED')
                    return (True, execute_result.status)
                else:
                    logger.error(f'EXECUTION FAILED, code: {execute_result.status}')
                    return (False, execute_result.status)
            else:
                logger.info('Planning Suceeded')
                return (True, 'PLAN_SUCEEDED')
        else:
            logger.error('Planning failed')
            return (False, 'PLAN_FAILED')



    def shutdown(self):
        self.thor.shutdown()


def main():

    ###################################################################
    # MoveItPy Setup
    ###################################################################
    rclpy.init()
    node=Node('oscar_test_node')
    oscar_moveit_py=Oscar(node)
    logger=get_logger('oscar_test_node')
    spin_thread=threading.Thread(target=rclpy.spin, args=(node,))
    spin_thread.start()
    ###########################################################################
    # Plan 1 - set states with predefined string
    ###########################################################################

    logger.info('Moving to pose "home" and closing grippers...')
    oscar_moveit_py.arm_go_to_named_pose('left', 'home')
    oscar_moveit_py.arm_go_to_named_pose('right', 'home')
    oscar_moveit_py.close_gripper('left')
    oscar_moveit_py.close_gripper('right')

    ###########################################################################
    # Plan 2 - set goal state with PoseStamped message
    ###########################################################################

    # set pose goal with PoseStamped message
    from tf_transformations import quaternion_from_euler
    from math import pi


    pose_goal = PoseStamped()
    orient = quaternion_from_euler(pi, 0, 0)
    pose_goal.header.frame_id = "world"
    pose_goal.pose.orientation.x = orient[0]
    pose_goal.pose.orientation.y = orient[1]
    pose_goal.pose.orientation.z = orient[2]
    pose_goal.pose.orientation.w = orient[3]
    pose_goal.pose.position.x = 0.4
    pose_goal.pose.position.y = 0.0
    pose_goal.pose.position.z = 0.85

    # call Oscar method
    logger.info('Moving to goal pose and openning gripper')
    pose_goal.pose.position.y=-0.25
    oscar_moveit_py.arm_go_to_pose('right', pose_goal)
    oscar_moveit_py.open_gripper('right')
    pose_goal.pose.position.y=0.25
    oscar_moveit_py.arm_go_to_pose('left', pose_goal)
    oscar_moveit_py.open_gripper('left')
    
    ###########################################################################
    # Plan 3 - set states with predefined string
    ###########################################################################

    logger.info('Moving to pose "home" and closing grippers...')
    oscar_moveit_py.arm_go_to_named_pose('left', 'home')
    oscar_moveit_py.arm_go_to_named_pose('right', 'home')
    oscar_moveit_py.close_gripper('left')
    oscar_moveit_py.close_gripper('right')

    try:
        while True:
            pass
    except:
        spin_thread.join()

