"""
A basic example of planning and executing a pose goal with moveit_py.
"""
import numpy as np
import rclpy
from rclpy.node import Node
import sys
import threading
import time

from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive

from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_py.core import RobotState
from moveit_py.planning import MoveItPy
from moveit_py.planning_interface import PlanningSceneInterface


def main():

    ###################################################################
    # MoveItPy Setup
    ###################################################################
    rclpy.init()
    logger = rclpy.logging.get_logger("moveit_py.planning_scene")

    # instantiate MoveItPy instance and get planning component
    moveit = MoveItPy(node_name="moveit_py")
    panda_arm = moveit.get_planning_component("panda_arm")

    ###############################################################################
    # Debug Planning Scene
    ###############################################################################
    logger.info("Debug Planning Scene")
    psm = moveit.get_planning_scene_monitor()
    logger.info(psm.name)

    # create collision object
    co = CollisionObject()
    co.operation = CollisionObject.ADD
    co.id = "box"
    co.header.frame_id = "panda_link0"

    box = SolidPrimitive()
    box.type = SolidPrimitive.BOX
    box.dimensions = [0.1, 0.1, 0.1]
    co.primitives = [box]

    pose = Pose()
    pose.position.x = 0.5
    pose.position.y = 0.5
    pose.position.z = 0.5
    pose.orientation.w = 1.0
    co.primitive_poses = [pose]

    # lock the planning scene and return read_write access to scene
    with psm.read_write() as scene:
        # add a collision object to the scene
        scene.apply_collision_object(co)

        # check if the current robot state is in collision
        robot_state = scene.get_current_state()
        robot_collision_status = scene.is_state_colliding(
            robot_state=robot_state, group="panda_arm", verbose=False
        )
        logger.info("Robot collision status: {}".format(robot_collision_status))

    ###############################################################################
    # Plan and Execute with Collision Object
    ###############################################################################

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

    with psm.read_write() as scene:
        # check collision status after adding object
        collision_status = scene.is_state_colliding(group="panda_arm", verbose=False)
        logger.info("Colliding status: {}".format(collision_status))

    # set plan start state using predefined state
    panda_arm.set_start_state_to_current_state()

    # set pose goal using predefined state
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
