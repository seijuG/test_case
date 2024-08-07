import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import PointCloud2
import numpy as np
from sensor_msgs_py import point_cloud2
import math

class PointCloudMerger(Node):
    def __init__(self):
        super().__init__('combined_attacks')
        qos_profile = QoSPresetProfiles.SENSOR_DATA.value

        # 新しいトピックのパブリッシャー
        self.publisher_raw = self.create_publisher(PointCloud2, '/sensing/lidar/top/pointcloud_raw', 10)
        self.publisher_raw_ex = self.create_publisher(PointCloud2, '/sensing/lidar/top/pointcloud_raw_ex', 10)

        # 既存トピックのサブスクライバー
        self.subscription_case3 = self.create_subscription(PointCloud2, '/sensing/lidar/top/pointcloud_case3', self.handle_case3, qos_profile)
        self.subscription_case4 = self.create_subscription(PointCloud2, '/sensing/lidar/top/pointcloud_case4', self.handle_case4, qos_profile)
        self.subscription_case3_ex = self.create_subscription(PointCloud2, '/sensing/lidar/top/pointcloud_raw_case3', self.handle_case3_ex, qos_profile)
        self.subscription_case4_ex = self.create_subscription(PointCloud2, '/sensing/lidar/top/pointcloud_raw_case4', self.handle_case4_ex, qos_profile)

        self.data = {}

    def handle_case3(self, data):
        self.data['case3'] = data
        self.publish_combined('raw')

    def handle_case4(self, data):
        self.data['case4'] = data
        self.publish_combined('raw')

    def handle_case3_ex(self, data):
        self.data['case3_ex'] = data
        self.publish_combined('raw_ex')

    def handle_case4_ex(self, data):
        self.data['case4_ex'] = data
        self.publish_combined('raw_ex')

    def publish_combined(self, key):
        if 'case3' in self.data and 'case4' in self.data and key == 'raw':
            self.combine_and_publish('/sensing/lidar/top/pointcloud_raw', 'case3', 'case4')
        elif 'case3_ex' in self.data and 'case4_ex' in self.data and key == 'raw_ex':
            self.combine_and_publish('/sensing/lidar/top/pointcloud_raw_ex', 'case3_ex', 'case4_ex')

    def combine_and_publish(self, topic, key1, key2):
        if key1 in self.data and key2 in self.data:
            fields = self.data[key1].fields
            points1 = np.array(list(point_cloud2.read_points(self.data[key1], field_names=[field.name for field in fields], skip_nans=True)))
            points2 = np.array(list(point_cloud2.read_points(self.data[key2], field_names=[field.name for field in fields], skip_nans=True)))

            combined_points = []
            for p in points1:
                angle = math.atan2(p[1], p[0])
                if -math.pi/6 <= angle <= math.pi/6:
                    combined_points.append(p)

            for p in points2:
                angle = math.atan2(p[1], p[0])
                if not (-math.pi/6 <= angle <= math.pi/6):
                    combined_points.append(p)

            combined_cloud = point_cloud2.create_cloud(self.data[key1].header, fields, combined_points)
            if topic == '/sensing/lidar/top/pointcloud_raw':
                self.publisher_raw.publish(combined_cloud)
            else:
                self.publisher_raw_ex.publish(combined_cloud)

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudMerger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

