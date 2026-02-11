import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped, TransformStamped
from nav_msgs.msg import Odometry   # RViz ve Navigasyon için standart mesaj
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster

# Webots ile konuşmak için gerekli QoS kütüphaneleri
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class GpsImuBridge(Node):

    def __init__(self):
        super().__init__('gps_bridge')

        # --- 1. QoS AYARLARI ---
        imu_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # --- 2. ABONELİKLER ---
        # GPS: Konum verisi (X, Y)
        self.gps_sub = self.create_subscription(
            PointStamped,
            '/GulseScan/gps',
            self.gps_callback,
            10
        )

        # IMU: Dönüş/Açı verisi (Orientation)
        self.imu_sub = self.create_subscription(
            Imu,
            '/GulseScan/imu',
            self.imu_callback,
            imu_qos_profile
        )

        # --- 3. YAYINCILAR ---
        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # Veri Saklama
        self.latest_imu = None

        self.get_logger().info('🚀 [GpsImuBridge] Başlatıldı! (QoS: Best Effort, Target: /GulseScan/imu)')

    def imu_callback(self, msg):
        """IMU verisi geldikçe hafızada tutuyoruz."""
        self.latest_imu = msg

    def gps_callback(self, msg):
        """GPS verisi geldiği an, en son IMU verisiyle birleştirip yayınlıyoruz."""

        if self.latest_imu is None:
            self.get_logger().warn('[SADECE GPS] IMU verisi bekleniyor...', throttle_duration_sec=2.0)
            return

        current_time = self.get_clock().now().to_msg()

        # --- A. TF (TRANSFORM) ---
        t = TransformStamped()
        t.header.stamp = current_time
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'

        t.transform.translation.x = msg.point.x
        t.transform.translation.y = msg.point.y
        t.transform.translation.z = 0.0
        t.transform.rotation = self.latest_imu.orientation

        self.tf_broadcaster.sendTransform(t)

        # --- B. ODOMETRY ---
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'

        odom.pose.pose.position.x = msg.point.x
        odom.pose.pose.position.y = msg.point.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = self.latest_imu.orientation

        self.odom_pub.publish(odom)

        self.get_logger().info(f'[IMU+GPS] Veri Akıyor -> X:{msg.point.x:.2f} Y:{msg.point.y:.2f}', once=True)


def main(args=None):
    rclpy.init(args=args)
    node = GpsImuBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
