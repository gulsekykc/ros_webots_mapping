import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class AutoNavigator(Node):
    def __init__(self):
        super().__init__('auto_navigator')
        self.subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info("🤖 Otonom Mod Aktif: Engel tanımam, harita çizerim!")

    def scan_callback(self, msg):
        twist = Twist()
        # Lidar verisini bölümlere ayırıyoruz (Örn: 360 derece için)
        # Orta (Ön), Sol ve Sağ bölgelerdeki en kısa mesafeleri buluyoruz
        front_dist = min(msg.ranges[160:200]) # Ön bölge
        left_dist = min(msg.ranges[200:300])  # Sol taraf
        right_dist = min(msg.ranges[60:160])  # Sağ taraf

        if front_dist < 0.6: # Engel çok yakınsa
            twist.linear.x = 0.0
            # Hangi taraf daha boşsa oraya dön ✨
            twist.angular.z = 1.0 if left_dist > right_dist else -1.0
            self.get_logger().info("🚧 Engel Var! Dönüyorum...")
        else:
            twist.linear.x = 0.3 # Yol boşsa ilerle
            twist.angular.z = 0.0
            
        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = AutoNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()