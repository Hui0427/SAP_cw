#!/usr/bin/env python3
import time
import math

import rclpy
from rclpy.logging import get_logger

from geometry_msgs.msg import PoseStamped
from moveit.planning import MoveItPy, PlanRequestParameters


def load_ply_points(path):
    """从 ASCII PLY 文件读取点 (x, y, z)。"""
    points = []
    with open(path, "r") as f:
        header_ended = False
        for line in f:
            line = line.strip()
            if not header_ended:
                if line == "end_header":
                    header_ended = True
                continue
            if not line:
                continue
            parts = line.split()
            if len(parts) < 3:
                continue
            try:
                x = float(parts[0])
                y = float(parts[1])
                z = float(parts[2])
                points.append((x, y, z))
            except ValueError:
                continue
    return points


def plan_and_execute(robot, planning_component, logger,
                     single_plan_parameters=None):
    """Plan + execute 一次运动。"""
    logger.info("Planning trajectory...")
    if single_plan_parameters is not None:
        plan_result = planning_component.plan(
            single_plan_parameters=single_plan_parameters
        )
    else:
        plan_result = planning_component.plan()

    if not plan_result:
        logger.error("Planning failed")
        return False

    logger.info("Executing trajectory...")
    robot_trajectory = plan_result.trajectory
    robot.execute(robot_trajectory, blocking=True, controllers=[])
    logger.info("Execution finished")
    return True


def make_pose(frame_id, x, y, z, qw=1.0, qx=0.0, qy=0.0, qz=0.0):
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    pose.pose.orientation.w = qw
    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    return pose


def main():
    rclpy.init()
    logger = get_logger("grasp_from_ply")

    # ===== 1) 读取 PLY 点云 =====
    ply_path = "/home/xinyue/ros2_ws/src/sfm/results/sfm_points_20251114_171953.ply"
    logger.info(f"Loading PLY from: {ply_path}")
    points = load_ply_points(ply_path)

    if not points:
        logger.error("No points loaded from PLY, aborting.")
        rclpy.shutdown()
        return

    logger.info(f"Loaded {len(points)} points from PLY")

    # 简单一点：用点云的质心 + 最高点，算一个抓取位置
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    zs = [p[2] for p in points]

    cx = sum(xs) / len(xs)
    cy = sum(ys) / len(ys)
    cz = sum(zs) / len(zs)

    # 找最高点
    max_z = max(zs)
    idx_top = zs.index(max_z)
    tx, ty, tz = points[idx_top]

    logger.info(f"Centroid: ({cx:.3f}, {cy:.3f}, {cz:.3f})")
    logger.info(f"Top point: ({tx:.3f}, {ty:.3f}, {tz:.3f})")

    # 这里我们取抓取点 = 最高点稍微往上 2cm
    grasp_x = tx
    grasp_y = ty
    grasp_z = tz + 0.02

    # 为了安全，可以限制一下工作空间范围（避免算出来太离谱）
    if math.sqrt(grasp_x**2 + grasp_y**2) > 1.0:
        logger.warn("Grasp point too far, clamping radius to 1.0m")
    logger.info(f"Chosen grasp point: ({grasp_x:.3f}, {grasp_y:.3f}, {grasp_z:.3f})")

    # 点云坐标系（你现在是 base_link）
    frame = "base_link"
    # 如果你在 URDF 里用的是 ee_link，就改成 "ee_link"
    ee_link = "tool0"

    # ===== 2) 初始化 MoveItPy =====
    ur5e = MoveItPy(node_name="grasp_from_ply")
    arm = ur5e.get_planning_component("ur_manipulator")
    logger.info("MoveItPy instance created for UR5e")

    try:
        params = PlanRequestParameters(ur5e)
    except TypeError:
        params = PlanRequestParameters(ur5e, "ur_manipulator")

    params.planning_pipeline = "ompl"
    params.planner_id = "RRTConnectkConfigDefault"
    params.max_velocity_scaling_factor = 0.2
    params.max_acceleration_scaling_factor = 0.2

    # ===== 3) 规划三个阶段：上方 → 抓取 → 抬起 =====
    approach_pose = make_pose(frame, grasp_x, grasp_y, grasp_z + 0.10)
    grasp_pose    = make_pose(frame, grasp_x, grasp_y, grasp_z)
    retreat_pose  = make_pose(frame, grasp_x, grasp_y, grasp_z + 0.20)

    # Step A: 先到物体上方
    logger.info("Moving to approach pose...")
    arm.set_start_state_to_current_state()
    arm.set_goal_state(pose_stamped_msg=approach_pose,
                       pose_link=ee_link)

    ok = plan_and_execute(ur5e, arm, logger, single_plan_parameters=params)
    if not ok:
        logger.error("Failed to reach approach pose, aborting.")
        rclpy.shutdown()
        return

    time.sleep(1.0)

    # Step B: 下到抓取位姿
    logger.info("Moving to grasp pose...")
    arm.set_start_state_to_current_state()
    arm.set_goal_state(pose_stamped_msg=grasp_pose,
                       pose_link=ee_link)

    ok = plan_and_execute(ur5e, arm, logger, single_plan_parameters=params)
    if not ok:
        logger.error("Failed to reach grasp pose, aborting.")
        rclpy.shutdown()
        return

    time.sleep(1.0)
    # 这里本来可以控制夹爪闭合（如果你有 gripper）

    # Step C: 把物体抬起来
    logger.info("Moving to retreat pose with grasped object...")
    arm.set_start_state_to_current_state()
    arm.set_goal_state(pose_stamped_msg=retreat_pose,
                       pose_link=ee_link)

    ok = plan_and_execute(ur5e, arm, logger, single_plan_parameters=params)
    if not ok:
        logger.error("Failed to reach retreat pose.")
        rclpy.shutdown()
        return

    logger.info("Grasp from PLY demo finished.")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
