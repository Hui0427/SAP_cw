from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Use ur_moveit_config package - MoveItConfigsBuilder will automatically find:
    # - ompl_planning.yaml, chomp_planning.yaml, etc. from ur_moveit_config/config/
    # Use URDF/SRDF from our own config with proper macro parameters (name and ur_type)
    moveit_demo_path = get_package_share_directory("moveit_demo")
    
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="ur5e", package_name="ur_moveit_config"
        )
        .robot_description(
            file_path=os.path.join(moveit_demo_path, "config", "ur5e.urdf.xacro"),
            mappings={"name": "ur5e", "ur_type": "ur5e"}
        )
        .robot_description_semantic(
            file_path=os.path.join(moveit_demo_path, "config", "ur5e.srdf.xacro"),
            mappings={"name": "ur5e"}
        )
        .moveit_cpp(
            file_path=os.path.join(moveit_demo_path, "config", "moveit_py.yaml")
        )
        .to_moveit_configs()
    )
    moveit_demo_params = moveit_config.to_dict()
    
    return LaunchDescription([
        Node(
            package="moveit_demo",
            executable="moveit_demo",
            name="moveit_demo",
            output="screen",
            emulate_tty=True,
            parameters=[moveit_demo_params],
        )
    ])

