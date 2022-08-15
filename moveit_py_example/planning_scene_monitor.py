"""
A basic example of interacting with the planning scene using moveit_py.
"""
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, PoseStamped
from shape_msgs.msg import SolidPrimitive

from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, PlanningScene
from moveit_py.core import RobotState
from moveit_py.planning import MoveItPy


def main():

    ###################################################################
    # MoveItPy Setup
    ###################################################################
    rclpy.init()
    logger = rclpy.logging.get_logger("moveit_py.planning_scene")

    # instantiate MoveItPy instance and get planning component
    moveit = MoveItPy(node_name="moveit_py")
    panda_arm = moveit.get_planning_component("panda_arm")

    # define box that we will use throughout tutorial
    box = SolidPrimitive()
    box.type = SolidPrimitive.BOX
    box.dimensions = [0.1, 0.1, 0.1]

    # define flattened box that we will use throughout tutorial
    box_flattened = SolidPrimitive()
    box_flattened.type = SolidPrimitive.BOX
    box_flattened.dimensions = [0.5, 0.8, 0.01]

    ###############################################################################
    # Plan and Execute with Collision Object in Scene
    ###############################################################################

    # get planning scene monitor instance from moveit_py instance
    psm = moveit.get_planning_scene_monitor()
    logger.info(psm.name)

    # create collision object
    co = CollisionObject()
    co.operation = CollisionObject.ADD
    co.id = "box_1"
    co.header.frame_id = "panda_link0"
    co.primitives = [box_flattened]
    pose = Pose()
    pose.position.x = 0.45
    pose.position.y = 0.35
    pose.position.z = 0.8
    co.primitive_poses = [pose]

    # lock the planning scene and return read_write access to a planning scene
    with psm.read_write() as scene:

        # add a collision object to the scene
        scene.apply_collision_object(co)

        # check if the current robot state is in collision
        robot_state = scene.current_state
        robot_state.update()  # required to update transforms
        robot_collision_status = scene.is_state_colliding(
            robot_state=robot_state, group="panda_arm", verbose=False
        )
        logger.info(
            "Robot collision status (current state): {}".format(robot_collision_status)
        )

        # check if a user-defined robot state is in collision
        pose_goal = Pose()
        pose_goal.position.x = 0.25
        pose_goal.position.y = 0.25
        pose_goal.position.z = 0.0
        pose_goal.orientation.w = 1.0
        robot_state.set_from_ik("panda_arm", pose_goal, "panda_link8", timeout=5.0)
        robot_state.update()  # required to update transforms

        # This robot state should be in collision
        robot_collision_status = scene.is_state_colliding(
            robot_state=robot_state, group="panda_arm", verbose=False
        )
        logger.info(
            "Robot collision status (user-defined robot state): {}".format(
                robot_collision_status
            )
        )

    # set plan start state using predefined state
    panda_arm.set_start_state("ready")

    # set pose goal using predefined state
    panda_arm.set_goal("extended")

    # plan to goal
    logger.info("Planning trajectory")
    plan_solution = panda_arm.plan()

    if plan_solution:
        logger.info("Executing plan")
        panda_arm.execute()

    ###############################################################################
    # Plan and Execute with Attached Collision Object
    ###############################################################################

    with psm.read_write() as scene:
        # attach collision object to robot
        a_co = AttachedCollisionObject()
        a_co.link_name = "panda_link8"
        a_co.touch_links = ["panda_link8", "panda_rightfinger", "panda_leftfinger"]

        co = CollisionObject()
        co.operation = CollisionObject.ADD
        co.id = "box_attached"
        co.header.frame_id = "panda_link0"
        co.primitives = [box]

        robot_state = scene.current_state
        robot_state.update()  # required to update transforms
        eef_pose = robot_state.get_pose("panda_link8")
        eef_pose.position.x += 0.15
        co.primitive_poses = [eef_pose]
        a_co.object = co
        scene.apply_attached_collision_object(a_co)

    # set plan start state to the current state
    panda_arm.set_start_state_to_current_state()

    # set pose goal with PoseStamped message
    pose_goal = PoseStamped()
    pose_goal.header.frame_id = "panda_link0"
    pose_goal.pose.orientation.w = 1.0
    pose_goal.pose.position.x = 0.28
    pose_goal.pose.position.y = -0.2
    pose_goal.pose.position.z = 0.2
    panda_arm.set_goal(goal_pose_msg=pose_goal, link_name="panda_link8")

    # plan to goal
    logger.info("Planning trajectory")
    plan_solution = panda_arm.plan()

    # Examine the plan solution
    solution_trajectory = plan_solution.trajectory
    logger.info(
        "The number of waypoints in solution: {}".format(
            solution_trajectory.num_waypoints
        )
    )
    logger.info("The duration of the solution: {}".format(solution_trajectory.duration))
    logger.info(
        "The average segment duration of solution: {}".format(
            solution_trajectory.average_segment_duration
        )
    )

    # lets add another object to the planning scene and check a waypoint for collisions
    waypoint = solution_trajectory.get_waypoint(
        solution_trajectory.num_waypoints - 1
    )  # get the second to last waypoint

    co2 = CollisionObject()
    co2.operation = CollisionObject.ADD
    co2.id = "box_2"
    co2.header.frame_id = "panda_link0"
    co2.primitives = [box_flattened]
    pose2 = Pose()
    pose2.position.x = 0.15
    pose2.position.y = 0.55
    pose2.position.z = 0.4
    co2.primitive_poses = [pose2]

    with psm.read_write() as scene:
        scene.apply_collision_object(co2)
        robot_collision_status = scene.is_state_colliding(
            robot_state=waypoint, group="panda_arm", verbose=False
        )
        logger.info("Waypoint collision status: {}".format(robot_collision_status))
        path_validity_status = scene.is_path_valid(
            path=plan_solution.trajectory, group="panda_arm"
        )
        logger.info("Path validity: {}".format(path_validity_status))

    # execute the plan if plan was successful and solution trajectory is not in collision with updated planning scene
    if bool(plan_solution) & (not robot_collision_status) & (path_validity_status):
        logger.info("Executing plan")
        panda_arm.execute()
    else:
        logger.info("Planning failed, replanning with updated planning scene")
        replan_solution = panda_arm.plan()
        if replan_solution:
            panda_arm.execute()
        else:
            logger.info("Replanning failed")

    ###############################################################################
    # Return to Ready Pose
    ###############################################################################

    # remove collision objects
    with psm.read_write() as scene:
        scene.remove_all_collision_objects()

    # set plan start state to the current state
    panda_arm.set_start_state_to_current_state()

    # set pose goal using predefined state
    panda_arm.set_goal("ready")

    # plan to goal
    logger.info("Planning trajectory")
    plan_solution = panda_arm.plan()

    # execute the plan
    if plan_solution:
        logger.info("Executing plan")
        panda_arm.execute()

    ###############################################################################
    # MoveItPy Shutdown
    ###############################################################################

    moveit.shutdown()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
