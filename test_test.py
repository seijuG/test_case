import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
import numpy as np
from sensor_msgs_py import point_cloud2
import math
import subprocess
import time
import rosbag2_py
import struct
import threading


class Case3(Node):
    def __init__(self):
        super().__init__('case3')
        qos_profile = QoSPresetProfiles.SENSOR_DATA.value

        # トピック1のパブリッシャーとサブスクライバー
        self.publisher1 = self.create_publisher(PointCloud2, '/sensing/lidar/top/pointcloud_raw', 10)
        self.subscription1 = self.create_subscription(
            PointCloud2,
            '/sensing/lidar/top/pointcloud_raw_add',
            self.modify_and_publish1,
            qos_profile)

        # トピック2のパブリッシャーとサブスクライバー
        self.publisher2 = self.create_publisher(PointCloud2, '/sensing/lidar/top/pointcloud_raw_ex', 10)
        self.subscription2 = self.create_subscription(
            PointCloud2,
            '/sensing/lidar/top/pointcloud_raw_ex_add',
            self.modify_and_publish2,
            qos_profile)

        # 前方方向の点群データを保持するリスト
        self.recorded_pointclouds_raw = []
        self.recorded_pointclouds_raw_ex = []
        self.load_pointclouds()
        self.current_idx_raw = 0
        self.current_idx_raw_ex = 0
        self.use_recorded_data = False
        self.dis_x = []
        self.dis_y = []
        self.dis_z = []
        self.dis_intensity = []
        self.dis_ring = []

    def load_pointclouds(self):
        storage_options = rosbag2_py.StorageOptions(uri='/home/nkoba/Desktop/rosbag2_2024_07_24-14_49_47/rosbag2_2024_07_24-14_49_47_0.db3', storage_id='sqlite3')
        converter_options = rosbag2_py.ConverterOptions("", "")
        reader = rosbag2_py.SequentialReader()
        reader.open(storage_options, converter_options)
        while reader.has_next():
            topic, data, timestamp = reader.read_next()
            if topic == '/sensing/lidar/top/pointcloud_raw':
                self.recorded_pointclouds_raw.append((data, timestamp))
                self.display_pointcloud_data(data)
            elif topic == '/sensing/lidar/top/pointcloud_raw_ex':
                self.recorded_pointclouds_raw_ex.append((data, timestamp))
                self.display_pointcloud_data(data)
        
    def display_pointcloud_data(self, data):
        self.dis_x = []
        self.dis_y = []
        self.dis_z = []
        self.dis_intensity = []
        self.dis_ring = []

        point_step = 24
        num_points = len(data) // point_step
        angle_range = 15

        for i in range(num_points):
            start = i * point_step
            end = start + point_step
            point_data = data[start:end]

            x, y, z = struct.unpack('<fff', point_data[0:12])
            intensity, = struct.unpack('<f', point_data[16:20])
            ring, = struct.unpack('<H', point_data[20:22])
            angle = math.degrees(math.atan2(y, x))
            if -angle_range <= angle <= angle_range:
                self.dis_x.append(x)
                self.dis_y.append(y)
                self.dis_z.append(z)
                self.dis_intensity.append(intensity)
                self.dis_ring.append(ring)

    def modify_and_publish1(self, data):
        assert isinstance(data, PointCloud2)

        points = np.array(list(point_cloud2.read_points(data, field_names=("x", "y", "z", "intensity", "ring"), skip_nans=True)))
        augmented_points = points.tolist()

        for x, y, z, intensity, ring in zip(self.dis_x, self.dis_y, self.dis_z, self.dis_intensity, self.dis_ring):
            angle = math.atan2(y, x)
            if -math.pi / 12 <= angle <= math.pi / 12:
                new_point = [x, y, z, intensity, ring]
                augmented_points.append(new_point)

        augmented_cloud = point_cloud2.create_cloud(data.header, data.fields, augmented_points)
        self.publisher1.publish(augmented_cloud)

    def modify_and_publish2(self, data):
        assert isinstance(data, PointCloud2)

        points = np.array(list(point_cloud2.read_points(data, field_names=("x", "y", "z", "intensity", "ring", "azimuth", "distance", "return_type", "time_stamp"), skip_nans=True)))
        augmented_points = points.tolist()

        for x, y, z, intensity, ring in zip(self.dis_x, self.dis_y, self.dis_z, self.dis_intensity, self.dis_ring):
            angle = math.atan2(y, x)
            if -math.pi / 12 <= angle <= math.pi / 12:
                new_point = [x, y, z, intensity, ring, 0.0, 0.0, 0, 0]  # 必要なフィールドを追加
                augmented_points.append(new_point)

        augmented_cloud = point_cloud2.create_cloud(data.header, data.fields, augmented_points)
        self.publisher2.publish(augmented_cloud)


def main(args=None):
    rclpy.init(args=args)
    pointcloud_processor = Case3()
    rclpy.spin(pointcloud_processor)
    pointcloud_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
