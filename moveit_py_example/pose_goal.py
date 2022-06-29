"""
A basic example of planning and executing a pose goal with moveit_py.
"""

import rclpy
from rclpy.node import Node
import sys
import threading
import time

from geometry_msgs.msg import PoseStamped 
from moveit_py.planning import MoveItPy
from moveit_py.utils import get_param_file

def main():
    # initialise rclpy and logging
    rclpy.init()
    logger = rclpy.logging.get_logger('moveit_py.pose_goal')
    

    # when using launch file the below utility function fetches the parameter config
    # it is also possible to manually specify param file if using `ros2 run`
    config_path = get_param_file()
    
    # instantiate MoveItPy instance 
    moveit = MoveItPy("moveit_py", config_path)
    logger.info("Completed moveit_py initialization")
    
    # start the planning scene service
    moveit.provide_planning_scene_service()

    # set planning component and start state
    panda_arm = moveit.get_planning_component("panda_arm")
    panda_arm.set_start_state("ready")

    # set pose goal
    pose_goal = PoseStamped()
    pose_goal.header.frame_id = "panda_link0"
    pose_goal.pose.orientation.w = 1.0
    pose_goal.pose.position.x = 0.28
    pose_goal.pose.position.y = -0.2
    pose_goal.pose.position.z = 0.5
    panda_arm.set_goal(pose_goal, "panda_link8")

    # plan to goal
    logger.info("Planning trajectory")
    panda_arm.plan()

    # execute the plan
    logger.info("Executing plan")
    panda_arm.execute(True)

    # sleep for a while to allow for inspection of the moveit_py node 
    time.sleep(600)

    # shutdown
    rclpy.shutdown()

if __name__ == "__main__":
    main()
