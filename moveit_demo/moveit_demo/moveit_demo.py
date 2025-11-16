#!/usr/bin/env python3
"""
MoveItPy demo script for UR5e robot arm.
This script demonstrates joint space and Cartesian space planning using MoveItPy.
Should be launched via the launch file to get proper parameter loading.
"""

import time
import rclpy
from moveit.planning import MoveItPy, PlanRequestParameters
from rclpy.logging import get_logger
from geometry_msgs.msg import PoseStamped
from moveit.core.robot_state import RobotState
from moveit.core.kinematic_constraints import construct_joint_constraint
import math

def plan_and_execute(robot, arm, planning_group, logger, vel=None, accel=None, sleep_time=0.0):
    """Helper function to plan and execute a motion."""
    # PlanRequestParameters - try with just robot first (like working example)
    try:
        params = PlanRequestParameters(robot)
    except TypeError:
        # If that fails, try with planning group
        params = PlanRequestParameters(robot, planning_group)
    
    # Set planning pipeline explicitly - use OMPL (was the working version)
    params.planning_pipeline = "ompl"
    params.planner_id = "RRTConnectkConfigDefault"
    
    if vel is not None:
        params.max_velocity_scaling_factor = vel
    if accel is not None:
        params.max_acceleration_scaling_factor = accel

    logger.info("Planning trajectory")
    plan_result = arm.plan(params)

    # Execute the plan
    if plan_result:
        logger.info("Executing plan")
        robot_trajectory = plan_result.trajectory
        
        # Wait for execution to complete
        execution_status = robot.execute(robot_trajectory, controllers=[])
        logger.info(f"Execution status: {execution_status}")
        
        # Wait a bit for the robot state to update
        time.sleep(0.5)
    else:
        logger.error("Planning failed")
    
    time.sleep(sleep_time)


def main():
    rclpy.init()
    logger = get_logger("moveit_demo")
    
    # Initialize MoveItPy - parameters will come from launch file
    ur5e = MoveItPy(node_name="moveit_demo")
    ur5e_arm = ur5e.get_planning_component("ur_manipulator")
    
    logger.info("MoveItPy instance created")
    planning_group = "ur_manipulator"
    
    # ========================================================================
    # EXAMPLE 1: Joint Space Planning - Using Custom Joint Angles
    # ========================================================================
    # Type: JOINT CONTROL (Direct joint angle specification)
    # Method: Sets specific joint angles using RobotState
    # How it works: 
    #   - Sets goal to custom joint angles: [0, -0.6, 1.0, -0.4, 0, 0] (radians)
    #   - Joint order: [shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3]
    #   - This positions the arm forward and extended, clearly different from default
    #   - Planner computes trajectory in joint space
    #   - All joints move simultaneously to reach the target configuration
    #   - Fast and predictable, but doesn't control end-effector path
    logger.info("=== Example 1: Joint Space Planning - Custom Joint Configuration ===")
    logger.info("Type: JOINT CONTROL | Method: Custom joint angles using RobotState")
    ur5e_arm.set_start_state_to_current_state()
    
    # Create a custom joint configuration that positions arm forward and extended
    robot_model = ur5e.get_robot_model()
    robot_state = RobotState(robot_model)
    # Joint angles: [shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3]
    # This positions arm forward, extended, ready for Cartesian planning to (0.4, 0.0, 0.35)
    joint_values = [0.0, -0.6, 1.0, -0.4, 0.0, 0.0]
    robot_state.set_joint_group_positions("ur_manipulator", joint_values)
    joint_constraint = construct_joint_constraint(
        robot_state=robot_state,
        joint_model_group=robot_model.get_joint_model_group("ur_manipulator"),
    )
    ur5e_arm.set_goal_state(motion_plan_constraints=[joint_constraint])
    plan_and_execute(ur5e, ur5e_arm, planning_group, logger, vel=0.2, accel=0.2, sleep_time=2.0)
    
    # ========================================================================
    # EXAMPLE 2: Cartesian Space Planning - Pose Goal
    # ========================================================================
    # Type: CARTESIAN POSITION CONTROL (End-effector pose specification)
    # Method: Specifies target position and orientation of end-effector (tool0)
    # How it works:
    #   - Sets goal to a specific Cartesian pose (x, y, z, orientation)
    #   - MoveIt computes inverse kinematics (IK) to find joint angles
    #   - Planner generates trajectory that moves end-effector to target pose
    #   - End-effector path is controlled, but joint paths may vary
    #   - If IK fails, planning fails (pose unreachable)
    logger.info("=== Example 2: Cartesian Space Planning - Pose Goal ===")
    logger.info("Type: CARTESIAN POSITION | Method: PoseStamped message with position/orientation")
    ur5e_arm.set_start_state_to_current_state()
    
    # Create a target pose - position and orientation in base_link frame
    # Adjusted to be reachable from the custom joint configuration in Example 1
    pose_goal = PoseStamped()
    pose_goal.header.frame_id = "base_link"
    # Position: 40cm forward, 10cm to the side, 35cm up (reachable from Example 1 configuration)
    pose_goal.pose.position.x = 0.4
    pose_goal.pose.position.y = 0.1
    pose_goal.pose.position.z = 0.35
    # Orientation: pointing forward with end-effector down (standard orientation)
    pose_goal.pose.orientation.x = 0.0
    pose_goal.pose.orientation.y = 0.707
    pose_goal.pose.orientation.z = 0.0
    pose_goal.pose.orientation.w = 0.707
    
    ur5e_arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link="tool0")
    # Use lower velocity/acceleration for Cartesian movements to avoid path tolerance violations
    plan_and_execute(ur5e, ur5e_arm, planning_group, logger, vel=0.15, accel=0.15, sleep_time=2.0)
    
    # ========================================================================
    # EXAMPLE 3: Joint Space Planning - Return to Home
    # ========================================================================
    # Type: JOINT CONTROL (Direct joint angle specification)
    # Method: Uses predefined "home" state from SRDF file
    # How it works:
    #   - Returns robot to safe "home" position
    #   - Joint angles: [0, -1.57, 0, 0, 0, 0]
    #   - Demonstrates chaining movements using set_start_state_to_current_state()
    logger.info("=== Example 3: Joint Space Planning - Return to 'home' state ===")
    logger.info("Type: JOINT CONTROL | Method: Predefined state from SRDF")
    ur5e_arm.set_start_state_to_current_state()
    ur5e_arm.set_goal_state(configuration_name="home")
    # Use lower velocity/acceleration to avoid path tolerance violations
    plan_and_execute(ur5e, ur5e_arm, planning_group, logger, vel=0.2, accel=0.2, sleep_time=2.0)
    
    logger.info("Demo completed successfully!")
    logger.info("Summary:")
    logger.info("  - Example 1: Joint Space Planning (JOINT CONTROL)")
    logger.info("  - Example 2: Cartesian Space Planning (CARTESIAN POSITION)")
    logger.info("  - Example 3: Joint Space Planning - Return to home (JOINT CONTROL)")


if __name__ == "__main__":
    main()
