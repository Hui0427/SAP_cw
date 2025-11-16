#!/usr/bin/env python3

import os
import cv2
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import PoseStamped
import tf2_ros
import tf2_geometry_msgs
import json
import pickle
from datetime import datetime

def load_images_from_folder(folder):
    """Load images from a folder and return them as a list."""
    images = []
    for filename in sorted(os.listdir(folder)):
        if filename.lower().endswith(('.png', '.jpg', '.jpeg')):
            img = cv2.imread(os.path.join(folder, filename))
            if img is not None:
                images.append(img)
    return images

def extract_features(image):
    """Extract SIFT features from an image."""
    sift = cv2.SIFT_create(nfeatures=50)
    keypoints, descriptors = sift.detectAndCompute(image, None)
    return keypoints, descriptors

def match_features(desc1, desc2):
    """Match features between two descriptor sets."""
    bf = cv2.BFMatcher(cv2.NORM_L2, crossCheck=True)
    matches = bf.match(desc1, desc2)
    matches = sorted(matches, key=lambda x: x.distance)
    return matches

def estimate_motion(kp1, kp2, matches, K):
    """Estimate camera motion between two frames using Essential Matrix."""
    pts1 = np.float32([kp1[m.queryIdx].pt for m in matches])
    pts2 = np.float32([kp2[m.trainIdx].pt for m in matches])

    pts1 = np.ascontiguousarray(pts1, dtype=np.float32).reshape(-1, 1, 2)
    pts2 = np.ascontiguousarray(pts2, dtype=np.float32).reshape(-1, 1, 2)

    E, mask = cv2.findEssentialMat(pts1, pts2, K, method=cv2.RANSAC, prob=0.999, threshold=1.0)
    _, R, t, mask = cv2.recoverPose(E, pts1, pts2, K)
    return R, t

def triangulate_points(R, t, kp1, kp2, matches, K):
    """Triangulate 3D points from 2D correspondences."""
    pts1 = np.float32([kp1[m.queryIdx].pt for m in matches]).T
    pts2 = np.float32([kp2[m.trainIdx].pt for m in matches]).T

    # Create projection matrices
    proj_matrix1 = K @ np.hstack((np.eye(3), np.zeros((3, 1))))
    proj_matrix2 = K @ np.hstack((R, t.reshape(3, 1)))

    # Triangulate points
    points_4d = cv2.triangulatePoints(proj_matrix1, proj_matrix2, pts1, pts2)
    points_3d = points_4d[:3] / points_4d[3]
    return points_3d.T

def create_pointcloud2(points_3d):
    """Create a ROS2 PointCloud2 message from 3D points."""
    msg = PointCloud2()
    msg.header.stamp = rclpy.time.Time().to_msg()
    msg.header.frame_id = "camera_link"
    
    # Set up fields
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    
    # Set up data
    msg.is_bigendian = False
    msg.point_step = 12  # 3 * 4 bytes
    msg.row_step = msg.point_step * len(points_3d)
    msg.data = points_3d.astype(np.float32).tobytes()
    msg.width = len(points_3d)
    msg.height = 1
    msg.is_dense = True
    
    return msg

class SfMNode(Node):
    """ROS2 node for Structure from Motion."""
    
    def __init__(self):
        super().__init__('sfm_node')
        
        # Declare parameters
        self.declare_parameter('dataset_path', '~/ros2_ws/src/sfm/dataset/')
        self.declare_parameter('output_path', '~/ros2_ws/src/sfm/results/')
        self.declare_parameter('nfeatures', 50)
        self.declare_parameter('ransac_threshold', 1.0)
        self.declare_parameter('ransac_probability', 0.999)
        self.declare_parameter('show_visualization', True)
        self.declare_parameter('save_results', True)
        
        # Get parameters
        self.dataset_path = os.path.expanduser(self.get_parameter('dataset_path').get_parameter_value().string_value)
        self.output_path = os.path.expanduser(self.get_parameter('output_path').get_parameter_value().string_value)
        self.nfeatures = self.get_parameter('nfeatures').get_parameter_value().integer_value
        self.ransac_threshold = self.get_parameter('ransac_threshold').get_parameter_value().double_value
        self.ransac_probability = self.get_parameter('ransac_probability').get_parameter_value().double_value
        self.show_visualization = self.get_parameter('show_visualization').get_parameter_value().bool_value
        self.save_results = self.get_parameter('save_results').get_parameter_value().bool_value
        
        # Create publishers
        self.pointcloud_pub = self.create_publisher(PointCloud2, '/sfm/points', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/sfm/camera_pose', 10)
        
        # Camera intrinsic matrix (example values - should be calibrated)
        self.K = np.array([[800, 0, 320],
                          [0, 800, 240],
                          [0, 0, 1]], dtype=np.float32)
        
        # Initialize variables
        self.images = []
        self.points_3d = []
        self.camera_poses = []
        self.current_pose = np.eye(4)  # Identity matrix for first camera
        
        self.get_logger().info('SfM Node initialized')
        
        # Start processing
        self.process_images()
    
    def process_images(self):
        """Load and process images for SfM."""
        self.get_logger().info(f'Loading images from: {self.dataset_path}')
        
        # Load images
        self.images = load_images_from_folder(self.dataset_path)
        
        if len(self.images) < 2:
            self.get_logger().error('Not enough images for SfM. Need at least 2 images.')
            return
        
        self.get_logger().info(f'Loaded {len(self.images)} images')
        
        # Process image pairs
        for i in range(len(self.images) - 1):
            self.get_logger().info(f'Processing images {i} and {i+1}')
            
            # Extract features
            kp1, desc1 = extract_features(self.images[i])
            kp2, desc2 = extract_features(self.images[i+1])
            
            if desc1 is None or desc2 is None:
                self.get_logger().warn(f'No features found in image {i} or {i+1}')
                continue
            
            # Match features
            matches = match_features(desc1, desc2)

            if len(matches) < 8:  # Need at least 8 points for Essential Matrix
                self.get_logger().warn(f'Not enough matches between images {i} and {i+1}')
                continue
            
            # Estimate motion
            R, t = estimate_motion(kp1, kp2, matches, self.K)
            
            # Update camera pose
            # Create 4x4 transformation matrix
            T = np.eye(4)
            T[:3, :3] = R
            T[:3, 3] = t.flatten()
            self.current_pose = self.current_pose @ T
            self.camera_poses.append(self.current_pose.copy())
            
            # Triangulate points
            points_3d = triangulate_points(R, t, kp1, kp2, matches, self.K)
            
            # Filter points (remove points behind camera)
            valid_points = points_3d[points_3d[:, 2] > 0]
            self.points_3d.extend(valid_points)
            
            self.get_logger().info(f'Triangulated {len(valid_points)} points')
            
            # Publish point cloud
            if len(self.points_3d) > 0:
                pointcloud_msg = create_pointcloud2(np.array(self.points_3d))
                self.pointcloud_pub.publish(pointcloud_msg)
            
            # Publish camera pose
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = "world"
            pose_msg.pose.position.x = float(self.current_pose[0, 3])
            pose_msg.pose.position.y = float(self.current_pose[1, 3])
            pose_msg.pose.position.z = float(self.current_pose[2, 3])
            self.pose_pub.publish(pose_msg)
        
        # Visualize results
        if self.show_visualization:
            self.visualize_reconstruction()
    
    def visualize_reconstruction(self):
        """Visualize the 3D reconstruction."""
        if len(self.points_3d) == 0:
            self.get_logger().warn('No 3D points to visualize')
            return
        
        fig = plt.figure(figsize=(12, 8))
        ax = fig.add_subplot(111, projection='3d')
        
        # Plot 3D points
        points_array = np.array(self.points_3d)
        ax.scatter(points_array[:, 0], points_array[:, 1], points_array[:, 2], 
                  c='blue', s=1, alpha=0.6, label='3D Points')
        
        # Plot camera poses
        if len(self.camera_poses) > 0:
            poses_array = np.array(self.camera_poses)
            ax.scatter(poses_array[:, 0, 3], poses_array[:, 1, 3], poses_array[:, 2, 3], 
                      c='red', s=50, marker='^', label='Camera poses')
        
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title('3D Reconstruction from Structure from Motion')
        ax.legend()
        
        # Enable interactive, non-blocking display
        plt.ion()
        plt.show(block=False)
        plt.pause(0.1)
        
        self.get_logger().info(f'Reconstruction complete: {len(self.points_3d)} 3D points, {len(self.camera_poses)} camera poses')
        
        # Save results if requested
        if self.save_results:
            self.save_reconstruction_results()
    
    def save_reconstruction_results(self):
        """Save reconstruction results to files."""
        # Create output directory
        os.makedirs(self.output_path, exist_ok=True)
        
        # Create timestamp for unique filenames
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # Save 3D points as numpy array
        if len(self.points_3d) > 0:
            points_file = os.path.join(self.output_path, f'sfm_points_{timestamp}.npy')
            np.save(points_file, np.array(self.points_3d))
            self.get_logger().info(f'Saved 3D points to: {points_file}')
        
        # Save camera poses as numpy array
        if len(self.camera_poses) > 0:
            poses_file = os.path.join(self.output_path, f'sfm_poses_{timestamp}.npy')
            np.save(poses_file, np.array(self.camera_poses))
            self.get_logger().info(f'Saved camera poses to: {poses_file}')
        
        # Save reconstruction metadata
        metadata = {
            'timestamp': timestamp,
            'num_points': len(self.points_3d),
            'num_poses': len(self.camera_poses),
            'num_images': len(self.images),
            'parameters': {
                'nfeatures': self.nfeatures,
                'ransac_threshold': self.ransac_threshold,
                'ransac_probability': self.ransac_probability
            },
            'camera_intrinsics': self.K.tolist()
        }
        
        metadata_file = os.path.join(self.output_path, f'sfm_metadata_{timestamp}.json')
        with open(metadata_file, 'w') as f:
            json.dump(metadata, f, indent=2)
        self.get_logger().info(f'Saved metadata to: {metadata_file}')
        
        # Save visualization plot
        if self.show_visualization:
            plot_file = os.path.join(self.output_path, f'sfm_visualization_{timestamp}.png')
            plt.savefig(plot_file, dpi=300, bbox_inches='tight')
            self.get_logger().info(f'Saved visualization to: {plot_file}')
        
        self.get_logger().info(f'All results saved to: {self.output_path}')

def main(args=None):
    rclpy.init(args=args)
    node = SfMNode()
    
    try:
        # Process images and reconstruction
        node.process_images()
        
        # Wait a bit for any final publications
        rclpy.spin_once(node, timeout_sec=1.0)
        
        # Gracefully shutdown
        node.get_logger().info('SfM reconstruction completed. Shutting down gracefully...')
        
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user. Shutting down...')
    except Exception as e:
        node.get_logger().error(f'Error during reconstruction: {str(e)}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()