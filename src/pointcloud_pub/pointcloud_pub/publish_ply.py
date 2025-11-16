#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import struct

class PLYPublisher(Node):
    def __init__(self):
        super().__init__('ply_publisher')

        #  这里改成你的 PLY 文件路径
        self.ply_path = "/home/xinyue/ros2_ws/src/sfm/results/sfm_points_20251114_171953.ply"
        self.get_logger().info(f"Loading PLY from: {self.ply_path}")

        self.points = self.load_ply(self.ply_path)
        self.publisher_ = self.create_publisher(PointCloud2, 'sfm_points', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)

    def load_ply(self, path):
        points = []
        with open(path, "r") as f:
            header = True
            for line in f:
                line = line.strip()
                if header:
                    if line.startswith("end_header"):
                        header = False
                    continue
                if not line:
                    continue
                parts = line.split()
                if len(parts) < 3:
                    continue
                x, y, z = map(float, parts[:3])
                points.append((x, y, z))
        return points

    def timer_callback(self):
        msg = self.create_pointcloud2(self.points)
        self.publisher_.publish(msg)

    def create_pointcloud2(self, points):
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"  # 你可以换成 "world" 或别的

        msg.height = 1
        msg.width = len(points)

        msg.fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12  # 3 * 4 bytes
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True

        buffer = []
        for x, y, z in points:
            buffer.append(struct.pack('fff', x, y, z))
        msg.data = b"".join(buffer)
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = PLYPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
