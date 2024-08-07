import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import rosbag2_py
import struct
import math
import time
import threading

class PointCloudSwitcher(Node):
    def __init__(self):
        super().__init__('pointcloud_switcher')
        self.qos_profile = QoSPresetProfiles.SENSOR_DATA.value
        self.publisher_raw = self.create_publisher(PointCloud2, '/sensing/lidar/top/pointcloud_raw', 10)
        self.publisher_raw_ex = self.create_publisher(PointCloud2, '/sensing/lidar/top/pointcloud_raw_ex', 10)

        self.subscription_raw_add = self.create_subscription(
            PointCloud2,
            '/sensing/lidar/top/pointcloud_raw_add',
            self.raw_callback,
            self.qos_profile)

        self.subscription_raw_ex_add = self.create_subscription(
            PointCloud2,
            '/sensing/lidar/top/pointcloud_raw_ex_add',
            self.raw_ex_callback,
            self.qos_profile)

        self.recorded_pointclouds_raw = []
        self.recorded_pointclouds_raw_ex = []
        self.load_recorded_pointclouds()
        self.current_idx_raw = 0
        self.current_idx_raw_ex = 0
        self.use_recorded_data = False
        self.value_x = []
        self.value_y = []
        self.value_z = []
        

        self.timer = self.create_timer(0.1, self.timer_callback)

    def load_recorded_pointclouds(self):
        storage_options = rosbag2_py.StorageOptions(uri='/home/nkoba/Desktop/rosbag2_2024_07_24-14_49_47/rosbag2_2024_07_24-14_49_47_0.db3', storage_id='sqlite3')
        converter_options = rosbag2_py.ConverterOptions('', '')
        reader = rosbag2_py.SequentialReader()
        reader.open(storage_options, converter_options)
        while reader.has_next():
            topic, data, timestamp = reader.read_next()
            if topic == '/sensing/lidar/top/pointcloud_raw':
                self.recorded_pointclouds_raw.append((data, timestamp))
                print("Added PointCloud2 data for /sensing/lidar/top/pointcloud_raw:")
                self.display_pointcloud_data(data)
            elif topic == '/sensing/lidar/top/pointcloud_raw_ex':
                self.recorded_pointclouds_raw_ex.append((data, timestamp))
                print("Added PointCloud2 data for /sensing/lidar/top/pointcloud_raw_ex:")
                self.display_pointcloud_data(data)

    def display_pointcloud_data(self, data):
        point_step = 18  # 各点のデータサイズ（バイト単位）
        num_points = len(data) // point_step
        angle_range = 15  # 15度の範囲
        for i in range(num_points):
            point_data = data[i*point_step:(i+1)*point_step]
            x, y, z, intensity, ring = struct.unpack('ffffh', point_data[:18])
            angle = math.degrees(math.atan2(y, x))
            if -angle_range <= angle <= angle_range:
                #print(f"Point {i}: x={x}, y={y}, z={z}, intensity={intensity}")
                self.value_x.append(x)
                self.value_y.append(y)
                self.value_z.append(z)


    def switch_to_recorded_data(self):
        self.use_recorded_data = True
        self.start_time = time.time()

    def raw_callback(self, msg):
        if not self.use_recorded_data:
            self.publisher_raw.publish(msg)

    def raw_ex_callback(self, msg):
        if not self.use_recorded_data:
            self.publisher_raw_ex.publish(msg)

    def timer_callback(self):
        if self.use_recorded_data:
            current_time = time.time()
            if self.current_idx_raw < len(self.recorded_pointclouds_raw):
                msg_raw, timestamp_raw = self.recorded_pointclouds_raw[self.current_idx_raw]
                elapsed_time_raw = current_time - self.start_time
                record_elapsed_time_raw = (timestamp_raw - self.recorded_pointclouds_raw[0][1]) / 1e9

                if elapsed_time_raw >= record_elapsed_time_raw:
                    self.publisher_raw.publish(msg_raw)
                    self.current_idx_raw += 1

            if self.current_idx_raw_ex < len(self.recorded_pointclouds_raw_ex):
                msg_raw_ex, timestamp_raw_ex = self.recorded_pointclouds_raw_ex[self.current_idx_raw_ex]
                elapsed_time_raw_ex = current_time - self.start_time
                record_elapsed_time_raw_ex = (timestamp_raw_ex - self.recorded_pointclouds_raw_ex[0][1]) / 1e9

                if elapsed_time_raw_ex >= record_elapsed_time_raw_ex:
                    self.publisher_raw_ex.publish(msg_raw_ex)
                    self.current_idx_raw_ex += 1

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudSwitcher()

    def wait_for_switch():
        input("Press Enter to switch to recorded data...")
        node.switch_to_recorded_data()

    switch_thread = threading.Thread(target=wait_for_switch)
    switch_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        switch_thread.join()

if __name__ == '__main__':
    main()
