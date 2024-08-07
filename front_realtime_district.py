import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from rclpy.qos import QoSProfile
import rosbag2_py
import numpy as np
import time
import struct

class PointCloudReplacer(Node):
    def __init__(self):
        super().__init__('pointcloud_replacer')
        qos_profile = QoSProfile(depth=10)
        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            '/sensing/lidar/top/pointcloud_raw',
            self.pointcloud_callback,
            qos_profile)

        self.pointcloud_pub = self.create_publisher(
            PointCloud2,
            '/sensing/lidar/top/pointcloud_raw',
            qos_profile)

        self.current_pointcloud = None
        self.replacement_pointclouds = []
        self.load_recorded_pointclouds()
        self.current_idx = 0

        if self.replacement_pointclouds:
            first_time = self.replacement_pointclouds[0][1]
            self.start_time = time.time()
            self.timer = self.create_timer(0.1, self.timer_callback)

    def pointcloud_callback(self, msg):
        self.current_pointcloud = msg

    def load_recorded_pointclouds(self):
        storage_options = rosbag2_py.StorageOptions(uri='/home/nkoba/Desktop/rosbag2_2024_07_24-14_49_47/rosbag2_2024_07_24-14_49_47_0.db3', storage_id='sqlite3')
        converter_options = rosbag2_py.ConverterOptions('', '')
        reader = rosbag2_py.SequentialReader()
        reader.open(storage_options, converter_options)
        while reader.has_next():
            topic, data, timestamp = reader.read_next()
            if topic == '/sensing/lidar/top/pointcloud_raw':
                self.replacement_pointclouds.append((data, timestamp))

    def pointcloud_within_angle(self, pointcloud, min_angle, max_angle):
        # 点群データを解析し、指定された角度範囲内のポイントを抽出する
        fields = pointcloud.fields
        field_names = [field.name for field in fields]

        cloud_data = np.frombuffer(pointcloud.data, dtype=np.float32).reshape(-1, len(fields))
        angles = np.arctan2(cloud_data[:, field_names.index('y')], cloud_data[:, field_names.index('x')]) * 180 / np.pi

        mask = (angles >= min_angle) & (angles <= max_angle)
        return cloud_data[mask]

    def replace_points_within_angle(self, original, replacement, min_angle, max_angle):
        # 元の点群データの指定された角度範囲内のポイントを置き換える
        original_data = np.frombuffer(original.data, dtype=np.float32).reshape(-1, 4)
        replacement_data = self.pointcloud_within_angle(replacement, min_angle, max_angle)

        # 置き換える範囲のポイントを元のデータに適用
        angles = np.arctan2(original_data[:, 1], original_data[:, 0]) * 180 / np.pi
        mask = (angles >= min_angle) & (angles <= max_angle)

        original_data[mask] = replacement_data[:len(original_data[mask])]

        # 変更されたデータを再パック
        original.data = original_data.tobytes()
        return original

    def timer_callback(self):
        if self.current_idx < len(self.replacement_pointclouds):
            msg, timestamp = self.replacement_pointclouds[self.current_idx]
            elapsed_time = time.time() - self.start_time
            record_elapsed_time = (timestamp - self.replacement_pointclouds[0][1]) / 1e9

            if elapsed_time >= record_elapsed_time:
                if self.current_pointcloud is not None:
                    modified_pointcloud = self.replace_points_within_angle(
                        self.current_pointcloud, msg, -15, 15)
                    self.pointcloud_pub.publish(modified_pointcloud)
                else:
                    self.pointcloud_pub.publish(msg)
                self.current_idx += 1

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudReplacer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
