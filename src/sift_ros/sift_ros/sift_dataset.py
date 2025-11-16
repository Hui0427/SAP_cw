#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import os
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class SIFTFeatureMatcher(Node):
    def __init__(self):
        super().__init__('sift_dataset_matcher')
        self.image_pub = self.create_publisher(Image, '/sift_features/matched_image', 10)
        self.dataset_path = os.path.expanduser('~/ros2_ws/src/sift_ros/dataset/')
        self.bridge = CvBridge()
        self.sift = cv2.SIFT_create()
        self.load_images()

    def load_images(self):
        self.image_list = []
        for file in sorted(os.listdir(self.dataset_path)):
            if file.endswith(('.jpg', '.png', '.jpeg')):
                image_path = os.path.join(self.dataset_path, file)
                image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
                if image is not None:
                    self.image_list.append((file, image))
        self.get_logger().info(f"Loaded {len(self.image_list)} images from dataset.")

    def process_images(self):
        if len(self.image_list) < 2:
            self.get_logger().error("Not enough images for feature matching.")
            return

        for i in range(len(self.image_list) - 1):
            img1_name, img1 = self.image_list[i]
            img2_name, img2 = self.image_list[i + 1]
            self.get_logger().info(f"Processing {img1_name} and {img2_name}")

            keypoints1, descriptors1 = self.sift.detectAndCompute(img1, None)
            keypoints2, descriptors2 = self.sift.detectAndCompute(img2, None)

            index_params = dict(algorithm=1, trees=5)
            search_params = dict(checks=50)
            flann = cv2.FlannBasedMatcher(index_params, search_params)

            matches = flann.knnMatch(descriptors1, descriptors2, k=2)
            good_matches = []
            for m, n in matches:
                if m.distance < 0.75 * n.distance:
                    good_matches.append(m)

            matched_image = cv2.drawMatches(img1, keypoints1, img2, keypoints2, good_matches, None)

            try:
                ros_image = self.bridge.cv2_to_imgmsg(matched_image, 'bgr8')
                self.image_pub.publish(ros_image)
            except Exception as e:
                self.get_logger().error(f"Failed to publish image: {str(e)}")

            rclpy.spin_once(self, timeout_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = SIFTFeatureMatcher()
    try:
        node.process_images()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
