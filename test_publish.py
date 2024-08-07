import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import PointCloud2
import numpy as np
from sensor_msgs_py import point_cloud2
import math
import rosbag2_py
import struct

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
        #self.dis_x = []
        #self.dis_y = []
        #self.dis_z = []

        #db3からデータをすべて取り出す
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
            #elif topic == '/sensing/lidar/top/pointcloud_raw_ex':
                #self.recorded_pointclouds_raw_ex.append((data, timestamp))
                #self.display_pointcloud_data(data)

    def display_pointcloud_data(self, data):
        #self.dis_x = []
        global dx 
        dx = []
        #self.dis_y = []
        global dy
        dy = []
        #self.dis_z = []
        global dz
        dz = []
        self.dis_intensity = []
        self.dis_ring = []

        point_step = 24
        num_points = len(data) // point_step
        angle_range = 90

        for i in range(num_points):
            start = i * point_step
            end = start + point_step
            point_data = data[start:end]

            x, y, z = struct.unpack('<fff', point_data[0:12])
            intensity = struct.unpack('<f', point_data[16:20])[0]
            ring = struct.unpack('<H', point_data[20:22])[0]
            angle = math.degrees(math.atan2(y, x))
            if -angle_range <= angle <= angle_range:
                #self.dis_x.append(x)
                dx.append(x)
                #self.dis_y.append(y)
                dy.append(y)
                #self.dis_z.append(z)
                dz.append(z)
                #self.dis_intensity.append(intensity)
                #self.dis_ring.append(ring)
                #print(dx)
    def modify_and_publish1(self, data):
        assert isinstance(data, PointCloud2)
        #print(self.dis_x)

        points = np.array(list(point_cloud2.read_points(data, field_names=("x", "y", "z", "intensity", "ring"), skip_nans=True)))
        augmented_points = points.tolist()

        for i, p in enumerate(points):
            if i < len(dx):
                angle_range = 90
                angle = math.degrees(math.atan2(p[1], p[0]))
                if -angle_range <= angle <= angle_range:
                    new_x = dx[i]
                    new_y = dy[i]
                    new_z = dz[i]
                    new_point = [new_x, new_y, new_z, p[3], p[4]]
                    augmented_points.append(new_point)

        augmented_cloud = point_cloud2.create_cloud(data.header, data.fields, augmented_points)
        self.publisher1.publish(augmented_cloud)

    def modify_and_publish2(self, data):
        assert isinstance(data, PointCloud2)

        points = np.array(list(point_cloud2.read_points(data, field_names=("x", "y", "z", "intensity", "ring", "azimuth", "distance", "return_type", "time_stamp"), skip_nans=True)))
        augmented_points = points.tolist()
        #print("1")
        for i, p in enumerate(points):
            if i < len(dx):
                angle_range = 90
                angle = math.degrees(math.atan2(p[1], p[0]))
                if -angle_range <= angle <= angle_range:
                    new_x = dx[i]
                    new_y = dy[i]
                    new_z = dz[i]
                    new_point = [new_x, new_y, new_z, p[3], p[4], p[5], p[6], p[7], p[8]]
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
