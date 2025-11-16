#!/usr/bin/env python3
import numpy as np

def debug_pointcloud():
    ply_path = "/home/xinyue/ros2_ws/src/sfm/results/sfm_points_20251114_171953.ply"
    
    points = []
    with open(ply_path, "r") as f:
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
                x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
                points.append((x, y, z))
            except ValueError:
                continue
    
    if points:
        points_array = np.array(points)
        min_vals = np.min(points_array, axis=0)
        max_vals = np.max(points_array, axis=0)
        bbox_size = max_vals - min_vals
        centroid = np.mean(points_array, axis=0)
        
        print("=== 点云诊断信息 ===")
        print(f"总点数: {len(points)}")
        print(f"X范围: [{min_vals[0]:.2f}, {max_vals[0]:.2f}]")
        print(f"Y范围: [{min_vals[1]:.2f}, {max_vals[1]:.2f}]") 
        print(f"Z范围: [{min_vals[2]:.2f}, {max_vals[2]:.2f}]")
        print(f"边界框尺寸: {bbox_size}")
        print(f"质心位置: {centroid}")
        print(f"最大维度: {np.max(bbox_size):.2f} 单位")
        
        # 检查点云是否在合理范围内
        if np.max(bbox_size) > 10.0:
            print("❌ 点云过大，需要缩放")
            suggested_scale = 0.5 / np.max(bbox_size)
            print(f"建议缩放因子: {suggested_scale:.6f}")
        else:
            print("✅ 点云尺寸合理")
            
        return points_array
    return None

if __name__ == "__main__":
    debug_pointcloud()
