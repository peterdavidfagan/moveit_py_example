"""
A basic example of planning and executing a pose goal with moveit_py.
"""
import numpy as np
import rclpy
from rclpy.node import Node
import sys
import threading
import time

from geometry_msgs.msg import PoseStamped
from moveit_py.planning import MoveItPy
from moveit_py.core import RobotState

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import yaml


def main():

    ###################################################################
    # MoveItPy Setup
    ###################################################################
    rclpy.init()
    logger = rclpy.logging.get_logger("moveit_py.pose_goal")

    # instantiate MoveItPy instance and get planning component
    moveit = MoveItPy(node_name="moveit_py")
    panda_arm = moveit.get_planning_component("panda_arm")

    ###########################################################################
    # Plan 1
    ###########################################################################

    # set plan start state using predefined state
    panda_arm.set_start_state("ready")

    # set pose goal using predefined state
    panda_arm.set_goal("extended")

    # plan to goal
    logger.info("Planning trajectory")
    plan_result = panda_arm.plan()

    # execute the plan
    if plan_result:
        logger.info("Executing plan")
        panda_arm.execute()

    ###########################################################################
    # Plan 2
    ###########################################################################

    # set plan start state to current state
    panda_arm.set_start_state_to_current_state()

    # set pose goal with PoseStamped message
    pose_goal = PoseStamped()
    pose_goal.header.frame_id = "panda_link0"
    pose_goal.pose.orientation.w = 1.0
    pose_goal.pose.position.x = 0.28
    pose_goal.pose.position.y = -0.2
    pose_goal.pose.position.z = 0.5
    panda_arm.set_goal(pose_goal, "panda_link8")

    # plan to goal
    logger.info("Planning trajectory")
    plan_result = panda_arm.plan()

    # execute the plan
    if plan_result:
        logger.info("Executing plan")
        panda_arm.execute()

    ###########################################################################
    # Plan 3
    ###########################################################################

    # instantiate a RobotState instance using the current robot model
    robot_model = moveit.get_robot_model()
    robot_state = RobotState(robot_model)

    # randomize the robot state
    robot_state.set_to_random_positions()

    # set plan start state to current state
    panda_arm.set_start_state_to_current_state()

    # set goal state to the initialized robot state
    logger.info("Set goal state to the initialized robot state")
    panda_arm.set_goal(robot_state)

    # plan to goal
    logger.info("Planning trajectory")
    plan_result = panda_arm.plan()

    # execute the plan
    if plan_result:
        logger.info("Executing plan")
        panda_arm.execute()

    ###########################################################################
    # Inspect Current RobotState
    ###########################################################################
    logger.info(robot_state.print_state_info())

    ###########################################################################
    # Plan 4
    ###########################################################################

    # set plan start state to current state
    panda_arm.set_start_state_to_current_state()

    # set goal state to the initialized robot state
    panda_arm.set_goal("ready")

    # plan to goal
    logger.info("Planning trajectory")
    plan_result = panda_arm.plan()

    # execute the plan
    if plan_result:
        logger.info("Executing plan")
        panda_arm.execute()

    ###############################################################################
    # MoveItPy Shutdown
    ###############################################################################

    moveit.shutdown()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
