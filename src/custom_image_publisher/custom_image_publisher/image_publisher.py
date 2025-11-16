#!/usr/bin/env python3
import os, glob, cv2, yaml
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

def load_camera_info(yaml_path: str, frame_id: str):
    if not yaml_path or not os.path.exists(yaml_path):
        return None
    with open(yaml_path, 'r') as f:
        calib = yaml.safe_load(f) or {}
    ci = CameraInfo()
    ci.header.frame_id = frame_id
    ci.width  = int(calib.get('image_width', 0) or 0)
    ci.height = int(calib.get('image_height', 0) or 0)
    ci.distortion_model = calib.get('distortion_model', 'plumb_bob') or 'plumb_bob'
    k = calib.get('camera_matrix', {}).get('data', [])
    ci.k = [float(x) for x in (k if len(k)==9 else [0.0]*9)]
    d = calib.get('distortion_coefficients', {}).get('data', [])
    ci.d = [float(x) for x in d]
    r = calib.get('rectification_matrix', {}).get('data', [])
    ci.r = [float(x) for x in r] if len(r)==9 else [1.0,0.0,0.0, 0.0,1.0,0.0, 0.0,0.0,1.0]
    p = calib.get('projection_matrix', {}).get('data', [])
    if len(p)==12:
        ci.p = [float(x) for x in p]
    else:
        K = ci.k
        ci.p = [K[0],0.0,K[2],0.0, 0.0,K[4],K[5],0.0, 0.0,0.0,1.0,0.0]
    return ci

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.declare_parameter('dataset_dir', os.path.expanduser('~/calibration_dataset'))
        self.declare_parameter('pattern', '*.jpeg')
        self.declare_parameter('topic', '/camera/image_raw')
        self.declare_parameter('publish_rate_hz', 1.0)
        self.declare_parameter('frame_id', 'camera')
        self.declare_parameter('loop', False)
        self.declare_parameter('camera_info_yaml', '')

        dataset_dir = self.get_parameter('dataset_dir').get_parameter_value().string_value
        pattern     = self.get_parameter('pattern').get_parameter_value().string_value
        image_topic = self.get_parameter('topic').get_parameter_value().string_value
        rate        = float(self.get_parameter('publish_rate_hz').get_parameter_value().double_value)
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.loop     = bool(self.get_parameter('loop').get_parameter_value().bool_value)
        camera_info_yaml = self.get_parameter('camera_info_yaml').get_parameter_value().string_value

        qos = QoSProfile(depth=10,
                         reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         history=QoSHistoryPolicy.KEEP_LAST)
        self.pub_img = self.create_publisher(Image, image_topic, qos)
        self.pub_ci  = self.create_publisher(CameraInfo, '/camera/camera_info', qos)
        self.bridge  = CvBridge()
        self.base_ci = load_camera_info(camera_info_yaml, self.frame_id)

        if self.base_ci:
            self.get_logger().info(f'Loaded camera info: {camera_info_yaml}')
        elif camera_info_yaml:
            self.get_logger().warning(f'camera_info_yaml given but not found: {camera_info_yaml}')

        self.image_files = sorted(glob.glob(os.path.join(dataset_dir, pattern)))
        if not self.image_files:
            self.get_logger().error(f'No images found: {os.path.join(dataset_dir, pattern)}')
            self.timer = None
            return

        self.get_logger().info(f'Loaded {len(self.image_files)} images from {dataset_dir}')
        self.index = 0
        period = max(1e-3, 1.0 / (rate if rate > 0 else 1.0))
        self.timer = self.create_timer(period, self.timer_cb)

    def make_ci_msg(self, stamp: Time):
        if not self.base_ci:
            return None
        ci = CameraInfo()
        ci.header.stamp = stamp
        ci.header.frame_id = self.frame_id
        ci.width, ci.height = self.base_ci.width, self.base_ci.height
        ci.distortion_model = self.base_ci.distortion_model
        ci.d = list(self.base_ci.d)
        ci.k = list(self.base_ci.k)
        ci.r = list(self.base_ci.r)
        ci.p = list(self.base_ci.p)
        return ci

    def timer_cb(self):
        if self.index >= len(self.image_files):
            if self.loop:
                self.get_logger().info('Looping back to start.')
                self.index = 0
            else:
                self.get_logger().info('Finished publishing all images.')
                if self.timer:
                    self.timer.cancel()
                return

        path = self.image_files[self.index]
        img = cv2.imread(path, cv2.IMREAD_COLOR)
        if img is None:
            self.get_logger().warning(f'Failed to load: {path}')
            self.index += 1
            return

        msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        now = self.get_clock().now().to_msg()
        msg.header.stamp = Time(sec=now.sec, nanosec=now.nanosec)
        msg.header.frame_id = self.frame_id
        self.pub_img.publish(msg)

        if self.base_ci:
            ci = self.make_ci_msg(msg.header.stamp)
            self.pub_ci.publish(ci)

        self.get_logger().info(f'Published [{self.index+1}/{len(self.image_files)}]: {os.path.basename(path)}')
        self.index += 1

def main():
    rclpy.init()
    node = ImagePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
