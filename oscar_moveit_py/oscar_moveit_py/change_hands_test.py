import rclpy
import numpy as np
from math import pi
from tf_transformations import quaternion_from_euler

from rclpy.node import Node
from oscar_moveit_py.oscar_moveit_py import Oscar

from oscar_interfaces.srv import ArmControl
from oscar_interfaces.srv import GripperControl
from geometry_msgs.msg import PoseStamped
from moveit.planning import PlanRequestParameters, PlanningComponent

class ChangeHandsTest(Node):

    def __init__():
        super().__init__('change_hands_node')


def main():
    rclpy.init()

    node=ChangeHandsTest()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()