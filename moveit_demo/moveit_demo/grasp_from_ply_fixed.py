#!/usr/bin/env python3
import time
import math
import numpy as np

import rclpy
from rclpy.logging import get_logger

from geometry_msgs.msg import PoseStamped
from moveit.planning import MoveItPy, PlanRequestParameters

def load_and_scale_ply_points(path):
    """加载PLY点云并自动缩放到合理尺寸"""
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
    
    if not points:
        return points
    
    # 转换为numpy数组
    points_array = np.array(points)
    
    # 计算点云尺寸
    min_vals = np.min(points_array, axis=0)
    max_vals = np.max(points_array, axis=0)
    bbox_size = max_vals - min_vals
    max_dimension = np.max(bbox_size)
    
    print(f"原始点云最大维度: {max_dimension:.2f} 单位")
    
    # 自动缩放：目标尺寸约0.3米
    if max_dimension > 1.0:
        scale_factor = 0.3 / max_dimension
        print(f"应用缩放因子: {scale_factor:.6f}")
        points_array = points_array * scale_factor
    else:
        print("点云尺寸合理，无需缩放")
    
    # 移动到机械臂前方
    centroid = np.mean(points_array, axis=0)
    target_center = np.array([0.4, 0.0, 0.2])  # 机械臂前方的安全位置
    translation = target_center - centroid
    translation[2] = 0  # 保持原始高度
    
    points_array = points_array + translation
    
    # 最终尺寸检查
    min_vals = np.min(points_array, axis=0)
    max_vals = np.max(points_array, axis=0)
    bbox_size = max_vals - min_vals
    print(f"处理后点云尺寸: {bbox_size}")
    print(f"处理后质心: {np.mean(points_array, axis=0)}")
    
    return [tuple(point) for point in points_array]

def is_point_in_workspace(x, y, z):
    """检查点是否在UR5e工作空间内"""
    radius = math.sqrt(x**2 + y**2)
    if radius > 0.8:
        return False
    if z > 1.2 or z < 0.1:
        return False
    return True

def find_grasp_point(points):
    """寻找抓取点"""
    if not points:
        return (0.4, 0.0, 0.3)  # 默认安全位置
    
    # 寻找工作空间内的最高点
    workspace_points = [p for p in points if is_point_in_workspace(p[0], p[1], p[2])]
    
    if workspace_points:
        workspace_points.sort(key=lambda p: p[2], reverse=True)
        return workspace_points[0]
    else:
        print("警告：没有找到工作空间内的点，使用默认位置")
        return (0.4, 0.0, 0.3)

def plan_and_execute(robot, planning_component, logger, single_plan_parameters=None):
    """执行运动规划"""
    logger.info("规划轨迹...")
    if single_plan_parameters is not None:
        plan_result = planning_component.plan(single_plan_parameters=single_plan_parameters)
    else:
        plan_result = planning_component.plan()

    if not plan_result:
        logger.error("规划失败")
        return False

    logger.info("执行轨迹...")
    robot_trajectory = plan_result.trajectory
    robot.execute(robot_trajectory, blocking=True, controllers=[])
    logger.info("执行完成")
    return True

def make_pose(frame_id, x, y, z):
    """创建位姿"""
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    # 垂直向下的抓取方向
    pose.pose.orientation.w = 0.707
    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.707
    pose.pose.orientation.z = 0.0
    return pose

def main():
    rclpy.init()
    logger = get_logger("grasp_from_ply")

    # 1. 加载并处理点云
    ply_path = "/home/xinyue/ros2_ws/src/sfm/results/sfm_points_20251114_171953.ply"
    logger.info("加载点云...")
    points = load_and_scale_ply_points(ply_path)

    if not points:
        logger.error("无法加载点云")
        rclpy.shutdown()
        return

    # 2. 选择抓取点
    grasp_x, grasp_y, grasp_z = find_grasp_point(points)
    grasp_z += 0.02  # 稍微高于表面
    
    logger.info(f"抓取点位置: ({grasp_x:.3f}, {grasp_y:.3f}, {grasp_z:.3f})")

    # 3. 初始化MoveIt
    try:
        logger.info("初始化MoveIt...")
        ur5e = MoveItPy(node_name="grasp_from_ply")
        arm = ur5e.get_planning_component("ur_manipulator")
    except Exception as e:
        logger.error(f"MoveIt初始化失败: {e}")
        logger.info("请确保：")
        logger.info("1. UR驱动正在运行")
        logger.info("2. MoveIt演示正在运行") 
        rclpy.shutdown()
        return

    # 配置规划参数
    try:
        params = PlanRequestParameters(ur5e)
    except TypeError:
        params = PlanRequestParameters(ur5e, "ur_manipulator")

    params.planning_pipeline = "ompl"
    params.planner_id = "RRTConnectkConfigDefault"
    params.max_velocity_scaling_factor = 0.2

    frame = "base_link"
    ee_link = "tool0"

    # 4. 执行抓取流程
    approach_pose = make_pose(frame, grasp_x, grasp_y, grasp_z + 0.10)
    grasp_pose = make_pose(frame, grasp_x, grasp_y, grasp_z)
    retreat_pose = make_pose(frame, grasp_x, grasp_y, grasp_z + 0.20)

    # 第一步：移动到预抓取位置
    logger.info("移动到预抓取位置...")
    arm.set_start_state_to_current_state()
    arm.set_goal_state(pose_stamped_msg=approach_pose, pose_link=ee_link)
    if not plan_and_execute(ur5e, arm, logger, params):
        logger.error("无法到达预抓取位置")
        rclpy.shutdown()
        return

    time.sleep(1.0)

    # 第二步：下降到抓取位置
    logger.info("下降到抓取位置...")
    arm.set_start_state_to_current_state()
    arm.set_goal_state(pose_stamped_msg=grasp_pose, pose_link=ee_link)
    if not plan_and_execute(ur5e, arm, logger, params):
        logger.error("无法到达抓取位置")
        rclpy.shutdown()
        return

    time.sleep(1.0)

    # 第三步：抬起
    logger.info("抬起物体...")
    arm.set_start_state_to_current_state()
    arm.set_goal_state(pose_stamped_msg=retreat_pose, pose_link=ee_link)
    if not plan_and_execute(ur5e, arm, logger, params):
        logger.error("无法抬起")
        rclpy.shutdown()
        return

    logger.info("🎉 抓取演示完成！")
    rclpy.shutdown()

if __name__ == "__main__":
    main()
