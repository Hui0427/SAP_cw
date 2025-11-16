#!/usr/bin/env python3
import os
import pathlib

def print_directory_structure(startpath, max_depth=5, current_depth=0, ignore_dirs=None):
    """
    打印目录结构
    
    Args:
        startpath: 起始路径
        max_depth: 最大递归深度
        current_depth: 当前深度（内部使用）
        ignore_dirs: 要忽略的目录列表
    """
    if ignore_dirs is None:
        ignore_dirs = ['.git', '__pycache__', 'build', 'install', 'log', '.cache']
    
    if current_depth > max_depth:
        return
        
    # 确保路径存在
    if not os.path.exists(startpath):
        print(f"路径不存在: {startpath}")
        return
    
    try:
        entries = os.listdir(startpath)
    except PermissionError:
        print(f"  {'  ' * current_depth} [权限拒绝]")
        return
    
    # 排序：先目录后文件，按字母顺序
    dirs = []
    files = []
    for entry in entries:
        if entry in ignore_dirs:
            continue
        full_path = os.path.join(startpath, entry)
        if os.path.isdir(full_path):
            dirs.append(entry)
        else:
            files.append(entry)
    
    dirs.sort()
    files.sort()
    
    # 打印当前目录内容
    for dir_name in dirs:
        full_path = os.path.join(startpath, dir_name)
        print(f"  {'  ' * current_depth}📁 {dir_name}/")
        print_directory_structure(full_path, max_depth, current_depth + 1, ignore_dirs)
    
    for file_name in files:
        file_path = os.path.join(startpath, file_name)
        file_size = os.path.getsize(file_path) if os.path.isfile(file_path) else 0
        size_str = f" ({file_size} bytes)" if file_size > 0 else ""
        
        # 根据文件类型添加不同的图标
        if file_name.endswith('.py'):
            icon = "🐍"
        elif file_name.endswith(('.cpp', '.h', '.hpp')):
            icon = "⚙️"
        elif file_name.endswith(('.xml', '.yaml', '.yml')):
            icon = "📋"
        elif file_name.endswith(('.md', '.txt')):
            icon = "📄"
        elif file_name.endswith(('.ply', '.pcd')):
            icon = "☁️"
        else:
            icon = "📄"
            
        print(f"  {'  ' * current_depth}{icon} {file_name}{size_str}")

def main():
    print("=" * 60)
    print("ROS2 工作空间文件结构")
    print("=" * 60)
    
    # 检查常见的ROS2工作空间路径
    possible_paths = [
        os.path.expanduser("~/ros2_ws"),
        os.path.expanduser("~/ros2_ws/src"),
        os.path.expanduser("~/ros_ws"),
        os.path.expanduser("~/catkin_ws"),
        os.path.abspath(".")  # 当前目录
    ]
    
    # 找到存在的路径
    valid_paths = [p for p in possible_paths if os.path.exists(p)]
    
    if not valid_paths:
        print("未找到ROS工作空间，使用当前目录:")
        start_path = os.path.abspath(".")
    else:
        print("找到以下工作空间:")
        for i, path in enumerate(valid_paths, 1):
            print(f"  {i}. {path}")
        
        choice = input(f"选择要显示的工作空间 (1-{len(valid_paths)}, 默认1): ").strip()
        if choice and choice.isdigit() and 1 <= int(choice) <= len(valid_paths):
            start_path = valid_paths[int(choice) - 1]
        else:
            start_path = valid_paths[0]
    
    print(f"\n显示路径: {start_path}")
    print("-" * 60)
    
    print_directory_structure(start_path)
    
    print("\n" + "=" * 60)
    print("关键文件检查:")
    print("=" * 60)
    
    # 检查关键文件
    key_files = [
        "src/pointcloud_pub/pointcloud_pub/publish_ply.py",
        "src/pointcloud_pub/setup.py",
        "src/pointcloud_pub/package.xml",
        "src/sfm/results/sfm_points_20251114_171953.ply"
    ]
    
    for rel_path in key_files:
        full_path = os.path.join(start_path, rel_path)
        if os.path.exists(full_path):
            size = os.path.getsize(full_path)
            print(f"✅ {rel_path} ({size} bytes)")
        else:
            print(f"❌ {rel_path} (不存在)")

if __name__ == "__main__":
    main()
