#!/usr/bin/env python3

import os
import cv2
import numpy as np
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose
import json
import pickle
from datetime import datetime

def load_images_from_folder(folder):
    """Load images from a folder and return them as a list of (filename, image)."""
    images = []
    for filename in sorted(os.listdir(folder)):
        if filename.lower().endswith(('.png', '.jpg', '.jpeg')):
            img = cv2.imread(os.path.join(folder, filename))
            if img is not None:
                images.append((filename, img))
    return images

class HomographyNode(Node):
    """ROS2 node for Homography computation."""
    
    def __init__(self):
        super().__init__('homography_node')
        
        # Declare parameters
        self.declare_parameter('dataset_path', '~/ros2_ws/src/homography/dataset/')
        self.declare_parameter('output_path', '~/ros2_ws/src/homography/results/')
        self.declare_parameter('max_features', 500)
        self.declare_parameter('match_ratio', 0.75)
        self.declare_parameter('ransac_threshold', 5.0)
        self.declare_parameter('warp_mode', 'perspective')  # 'perspective' or 'affine'
        self.declare_parameter('show_visualization', True)
        self.declare_parameter('save_results', True)
        
        # Get parameters
        self.dataset_path = os.path.expanduser(self.get_parameter('dataset_path').get_parameter_value().string_value)
        self.output_path = os.path.expanduser(self.get_parameter('output_path').get_parameter_value().string_value)
        self.max_features = self.get_parameter('max_features').get_parameter_value().integer_value
        self.match_ratio = self.get_parameter('match_ratio').get_parameter_value().double_value
        self.ransac_threshold = self.get_parameter('ransac_threshold').get_parameter_value().double_value
        self.warp_mode = self.get_parameter('warp_mode').get_parameter_value().string_value
        self.show_visualization = self.get_parameter('show_visualization').get_parameter_value().bool_value
        self.save_results = self.get_parameter('save_results').get_parameter_value().bool_value
        
        # Create publishers
        self.image_pub = self.create_publisher(Image, '/homography/warped_image', 10)
        self.pose_pub = self.create_publisher(Pose, '/homography/transformation', 10)
        
        # Initialize OpenCV bridge
        self.bridge = CvBridge()
        
        # Initialize variables
        self.images = []
        self.homography_data = []
        
        # Initialize SIFT detector
        self.sift = cv2.SIFT_create(nfeatures=self.max_features)
        
        # FLANN parameters
        FLANN_INDEX_KDTREE = 1
        index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
        search_params = dict(checks=50)
        self.flann = cv2.FlannBasedMatcher(index_params, search_params)
        
        self.get_logger().info('Homography Node initialized')
    
    def process_images(self):
        """Load and process images for homography computation."""
        self.get_logger().info(f'Loading images from: {self.dataset_path}')
        
        # Load images
        self.images = load_images_from_folder(self.dataset_path)
        
        if len(self.images) < 2:
            self.get_logger().error('Not enough images for homography. Need at least 2 images.')
            return
        
        self.get_logger().info(f'Loaded {len(self.images)} images')
        
        # Process image pairs
        for i in range(len(self.images) - 1):
            self.get_logger().info(f'Processing images {i} and {i+1}')
            name1, img1 = self.images[i]
            name2, img2 = self.images[i+1]
            
            # Compute homography between consecutive images
            H, matches, kp1, kp2 = self.compute_homography(img1, img2)
            
            if H is not None:
                # Warp images in both directions for clear interpretation
                # H maps img1 -> img2; use H_inv to map img2 -> img1
                H_inv = None
                try:
                    H_inv = np.linalg.inv(H)
                except Exception:
                    pass

                # img1 -> img2 frame
                warped_1to2 = self.warp_image(img1, H, self.warp_mode)
                # img2 -> img1 frame (preferred to visualize alignment onto first image)
                warped_2to1 = self.warp_image(img2, H_inv, self.warp_mode) if H_inv is not None else img2.copy()
                
                # Create visualization (matches view). Also create side-by-side original vs warped (2->1)
                vis_img = self.create_visualization(img1, img2, warped_2to1, matches, kp1, kp2)
                
                # Create original vs warped side-by-side for quick inspection
                side_warp = warped_2to1
                if img1.shape[:2] != side_warp.shape[:2]:
                    side_warp = cv2.resize(side_warp, (img1.shape[1], img1.shape[0]))
                side_by_side = np.hstack([img1, side_warp])
                
                # Store homography data (will be populated with output file paths below)
                homography_info = {
                    'frame_pair': f'{i}-{i+1}',
                    'image_pair': f'{name1}__{name2}',
                    'images': {
                        'image1': os.path.join(self.dataset_path, name1),
                        'image2': os.path.join(self.dataset_path, name2)
                    },
                    'homography_matrix': H.tolist(),
                    'num_matches': len(matches),
                    'inlier_ratio': len(matches) / max(len(kp1), len(kp2)) if max(len(kp1), len(kp2)) > 0 else 0,
                    'outputs': {}
                }

                # Optional on-screen visualization
                if self.show_visualization:
                    # Display matches view using matplotlib and block until window closed
                    plt.figure('Homography: original vs matches (left/right)')
                    plt.imshow(cv2.cvtColor(vis_img, cv2.COLOR_BGR2RGB))
                    plt.axis('off')
                    plt.tight_layout()
                    plt.show(block=True)
                    plt.close()

                # Save per-pair artifacts with image-based names
                if self.save_results:
                    safe1 = os.path.splitext(os.path.basename(name1))[0]
                    safe2 = os.path.splitext(os.path.basename(name2))[0]
                    pair_tag = f"{safe1}_to_{safe2}"
                    os.makedirs(self.output_path, exist_ok=True)

                    # Save visualization image
                    vis_path = os.path.join(self.output_path, f"homography_vis_{pair_tag}.png")
                    cv2.imwrite(vis_path, vis_img)
                    homography_info['outputs']['matches_visualization'] = vis_path
                    self.get_logger().info(f'Saved: {vis_path}')

                    # Save canonical warp names like previous convention
                    # <img1>-<img2>.png means second image warped into first image frame
                    legacy_2to1 = os.path.join(self.output_path, f"{safe1}-{safe2}.png")
                    legacy_1to2 = os.path.join(self.output_path, f"{safe2}-{safe1}.png")

                    # Save warped images
                    cv2.imwrite(legacy_2to1, warped_2to1)
                    cv2.imwrite(legacy_1to2, warped_1to2)
                    homography_info['outputs']['warped_2to1'] = legacy_2to1
                    homography_info['outputs']['warped_1to2'] = legacy_1to2
                    self.get_logger().info(f'Saved: {legacy_2to1}')
                    self.get_logger().info(f'Saved: {legacy_1to2}')

                    # Save original vs warped side-by-side image
                    side_path = os.path.join(self.output_path, f"original_vs_warped_{pair_tag}.png")
                    cv2.imwrite(side_path, side_by_side)
                    homography_info['outputs']['original_vs_warped'] = side_path
                    self.get_logger().info(f'Saved: {side_path}')

                    # Save homography matrix JSON per pair
                    H_path = os.path.join(self.output_path, f"H_{pair_tag}.json")
                    with open(H_path, 'w') as f:
                        json.dump({'H': H.tolist(), 'pair': [name1, name2]}, f, indent=2)
                    homography_info['outputs']['homography_json'] = H_path
                    self.get_logger().info(f'Saved: {H_path}')

                    # Save H as human-readable TXT (aligned columns)
                    H_txt_path = os.path.join(self.output_path, f"H_{pair_tag}.txt")
                    with open(H_txt_path, 'w') as f:
                        f.write("Homography matrix (mapping img1->img2)\n")
                        for row in H:
                            f.write("  " + "\t".join(f"{val: .6f}" for val in row) + "\n")
                    homography_info['outputs']['homography_txt'] = H_txt_path
                    self.get_logger().info(f'Saved: {H_txt_path}')

                    # Save H as CSV for spreadsheets
                    H_csv_path = os.path.join(self.output_path, f"H_{pair_tag}.csv")
                    try:
                        np.savetxt(H_csv_path, H, delimiter=",", fmt="%.8f")
                        homography_info['outputs']['homography_csv'] = H_csv_path
                        self.get_logger().info(f'Saved: {H_csv_path}')
                    except Exception as e:
                        self.get_logger().warn(f'Could not save CSV: {str(e)}')

                # Append pair info (including outputs) to summary list
                self.homography_data.append(homography_info)
                
                # Publish warped image
                try:
                    ros_image = self.bridge.cv2_to_imgmsg(vis_img, "bgr8")
                    self.image_pub.publish(ros_image)
                except Exception as e:
                    self.get_logger().error(f'Failed to publish image: {str(e)}')
                
                # Publish transformation as pose (simplified)
                pose_msg = Pose()
                # Extract rotation and translation from homography (simplified)
                pose_msg.position.x = H[0, 2]  # Translation x
                pose_msg.position.y = H[1, 2]  # Translation y
                pose_msg.position.z = 0.0
                # Convert rotation matrix to quaternion (simplified)
                pose_msg.orientation.w = 1.0
                pose_msg.orientation.x = 0.0
                pose_msg.orientation.y = 0.0
                pose_msg.orientation.z = 0.0
                self.pose_pub.publish(pose_msg)
                
                self.get_logger().info(f'Processed frame pair {i}-{i+1} with {len(matches)} matches')
            else:
                self.get_logger().warn(f'Could not compute homography for frame pair {i}-{i+1}')
        
        # Save summary results if requested
        if self.save_results:
            self.save_homography_results()
        
        self.get_logger().info('Homography processing completed')
    
    def compute_homography(self, img1, img2):
        """Compute homography between two images."""
        # Convert to grayscale
        gray1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
        gray2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)
        
        # Detect keypoints and descriptors
        kp1, des1 = self.sift.detectAndCompute(gray1, None)
        kp2, des2 = self.sift.detectAndCompute(gray2, None)
        
        if des1 is None or des2 is None:
            return None, None, kp1, kp2
        
        # Match features
        matches = self.flann.knnMatch(des1, des2, k=2)
        
        # Apply Lowe's ratio test
        good_matches = []
        for match_pair in matches:
            if len(match_pair) == 2:
                m, n = match_pair
                if m.distance < self.match_ratio * n.distance:
                    good_matches.append(m)
        
        if len(good_matches) < 10:
            return None, good_matches, kp1, kp2
        
        # Extract matched points
        src_pts = np.float32([kp1[m.queryIdx].pt for m in good_matches]).reshape(-1, 1, 2)
        dst_pts = np.float32([kp2[m.trainIdx].pt for m in good_matches]).reshape(-1, 1, 2)
        
        # Compute homography using RANSAC
        H, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, self.ransac_threshold)
        
        if H is not None:
            # Filter matches based on RANSAC mask
            inlier_matches = [good_matches[i] for i in range(len(good_matches)) if mask[i]]
            return H, inlier_matches, kp1, kp2
        else:
            return None, good_matches, kp1, kp2
    
    def warp_image(self, img, H, mode='perspective'):
        """Warp image using homography matrix."""
        h, w = img.shape[:2]
        
        if mode == 'perspective':
            # Apply full homography transformation
            warped = cv2.warpPerspective(img, H, (w, h))
        elif mode == 'affine':
            # Convert to affine transformation (first 2 rows)
            H_affine = H[:2, :]
            warped = cv2.warpAffine(img, H_affine, (w, h))
        else:
            warped = img
        
        return warped
    
    def create_visualization(self, img1, img2, warped_img, matches, kp1, kp2):
        """Create visualization of homography results."""
        # Create side-by-side visualization
        h1, w1 = img1.shape[:2]
        h2, w2 = img2.shape[:2]
        
        # Create canvas
        canvas_h = max(h1, h2)
        canvas_w = w1 + w2 + 50  # Extra space between images
        canvas = np.zeros((canvas_h, canvas_w, 3), dtype=np.uint8)
        
        # Place images side by side
        canvas[:h1, :w1] = img1
        canvas[:h2, w1+50:w1+50+w2] = img2
        
        # Draw matches
        if matches and len(matches) > 0:
            for match in matches[:50]:  # Limit to first 50 matches for clarity
                pt1 = tuple(map(int, kp1[match.queryIdx].pt))
                pt2 = tuple(map(int, kp2[match.trainIdx].pt))
                pt2 = (pt2[0] + w1 + 50, pt2[1])  # Adjust for side-by-side layout
                
                # Draw match line
                cv2.line(canvas, pt1, pt2, (0, 255, 0), 1)
                # Draw keypoints
                cv2.circle(canvas, pt1, 3, (0, 0, 255), -1)
                cv2.circle(canvas, pt2, 3, (0, 0, 255), -1)
        
        # Add text labels
        cv2.putText(canvas, 'Image 1', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.putText(canvas, 'Image 2', (w1 + 60, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.putText(canvas, f'Matches: {len(matches)}', (10, canvas_h - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        return canvas
    
    def save_homography_results(self):
        """Save homography results to files."""
        # Create output directory
        os.makedirs(self.output_path, exist_ok=True)
        
        # Create timestamp for summary file naming
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # Save homography data as JSON (pairs only)
        homography_file = os.path.join(self.output_path, f'homography_summary_{timestamp}.json')
        with open(homography_file, 'w') as f:
            json.dump({'pairs': self.homography_data}, f, indent=2)
        self.get_logger().info(f'Saved homography data to: {homography_file}')
        
        # Save metadata
        metadata = {
            'timestamp': timestamp,
            'warp_mode': self.warp_mode,
            'num_frames': len(self.images),
            'parameters': {
                'max_features': self.max_features,
                'match_ratio': self.match_ratio,
                'ransac_threshold': self.ransac_threshold
            }
        }
        
        metadata_file = os.path.join(self.output_path, f'homography_metadata_{timestamp}.json')
        with open(metadata_file, 'w') as f:
            json.dump(metadata, f, indent=2)
        self.get_logger().info(f'Saved metadata to: {metadata_file}')
        
        self.get_logger().info(f'All results saved to: {self.output_path}')

def main(args=None):
    rclpy.init(args=args)
    node = HomographyNode()
    
    try:
        # Process images and homography
        node.process_images()
        
        # Wait a bit for any final publications
        rclpy.spin_once(node, timeout_sec=1.0)
        
        # Gracefully shutdown
        node.get_logger().info('Homography processing completed. Shutting down gracefully...')
        
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user. Shutting down...')
    except Exception as e:
        node.get_logger().error(f'Error during processing: {str(e)}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
