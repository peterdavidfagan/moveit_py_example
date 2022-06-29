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

from moveit_msgs.msg import CollisionObject
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

    # instantiate planning scene interface
    planning_scene = PlanningSceneInterface()

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

    # add collision object to scene
    planning_scene.apply_collision_object(co)

    # get known object names
    object_names = planning_scene.get_known_object_names()
    logger.info(str(object_names))

    # get known objects in roi
    object_names = planning_scene.get_known_object_names_in_roi(
        min_x=-0.5, max_x=0.5, min_y=-0.5, max_y=0.5, min_z=-0.5, max_z=0.5, with_type=False
    )
    logger.info(str(object_names))


    # remove collision objects (updates asynchronously so not very useful)  
    #planning_scene.remove_collision_objects(["box"])

    ###############################################################################
    # Plan and Execute
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

    ###############################################################################
    # MoveItPy Shutdown
    ###############################################################################

    moveit.shutdown()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
