#!/usr/bin/env python3
from rclpy.node import Node
import rclpy

from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped
from tf_transformations import euler_from_quaternion, quaternion_from_euler, quaternion_matrix, quaternion_multiply

from er_crane_x7_srvs.srv import SetEEPose, SetGripper, SetGoalstate

import math
import numpy as np

class SimpleArmController(Node):
    def __init__(self, tf_buffer: Buffer=None):
        super().__init__('simple_arm_controller')

        self._cli_pose = self.create_client(SetEEPose, 'set_ee_pose')
        self._cli_grip = self.create_client(SetGripper, 'set_gripper')
        self._cli_gstate = self.create_client(SetGoalstate, 'move_groupstate')

        self.tf_buffer = Buffer() if tf_buffer is None else tf_buffer
        self.tf_listener = TransformListener(self.tf_buffer, self)

        while not self._cli_pose.wait_for_service(timeout_sec=10) or not self._cli_grip.wait_for_service(timeout_sec=10):
            self.get_logger().error('server is not running')

    def ee_abs_pose(self, x: float=0., y: float=0., z: float=0.2, yaw: float=90., pitch: float=0., roll: float=90., wait: bool=True) -> bool:
        ee_pose = SetEEPose.Request()
        ee_pose.x = x
        ee_pose.y = y
        ee_pose.z = z
        ee_pose.roll = float(roll)
        ee_pose.pitch = float(pitch)
        ee_pose.yaw = float(yaw)

        future = self._cli_pose.call_async(ee_pose)
        if wait: rclpy.spin_until_future_complete(self, future); return future.result().success
        return bool(future)
    
    def gripper(self, open: bool=True, angle: float=None, wait: bool=True) -> bool:
        if angle is None and open: angle = 90.0
        if angle is None and not open: angle = 0.0

        gripper = SetGripper.Request()
        gripper.angle = float(angle)

        future = self._cli_grip.call_async(gripper)
        if wait: rclpy.spin_until_future_complete(self, future); return future.result().success
        return bool(future)

    def goal_state(self, goal: str='home', max_vel: float=0.15, wait: bool=True) -> bool:
        goal_state = SetGoalstate.Request()
        goal_state.goal_state = goal
        goal_state.max_vel = max_vel
        future = self._cli_gstate.call_async(goal_state)
        if wait: rclpy.spin_until_future_complete(self, future); return future.result().result
        return bool(future)


def main(args=None):
    rclpy.init(args=args)
    arm_control = SimpleArmController()

    arm_control.ee_abs_pose(z=0.4)

if __name__ == '__main__':
    main()
