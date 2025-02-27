import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class TurtleBotController(Node):
    def __init__(self):
        super().__init__('turtlebot_controller')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            LaserScan, 
            '/scan', 
            self.lidar_callback, 
            10)
        self.subscription
        self.twist = Twist()
        
    def destroy_node(self):
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.publisher.publish(self.twist)  # ส่งคำสั่งหยุด
        self.get_logger().info("🛑 Stopping TurtleBot3 before shutdown")
        super().destroy_node()

    def lidar_callback(self, msg):
        ranges = msg.ranges
        angles = [0, 90, 180, 270]  # ทิศทางหลัก
        tolerance = 30  # องศาที่ให้การตรวจจับวัตถุ
        
        # กรองค่าที่ผิดพลาด (Infinity หรือ 0)
        valid_ranges = [r if r > 0 else float('inf') for r in ranges]

        # คำนวณระยะของวัตถุแต่ละทิศ (ถ้าไม่มีวัตถุ ให้ใช้ค่าสูงสุด 4.0)
        distances = {
            angle: min(valid_ranges[max(0, angle - tolerance): min(len(ranges), angle + tolerance + 1)], default=4.0)
            for angle in angles
        }

        # คำนวณความเร็ว: ใกล้ -> เร็ว, ไกล -> ช้า (ระยะ 0.2 - 4.0 เมตร)
        min_distance = min(distances.values())
        speed = max(0.1, min(1.5, (4.0 - min_distance) * 0.5))  # จำกัดความเร็วระหว่าง 0.1 - 1.5 m/s
        
        # กำหนดทิศทางการเคลื่อนที่
        if distances[0] < 4.0:  # วัตถุด้านหน้า -> เดินหน้า
            self.twist.linear.x = speed
            self.twist.angular.z = 0.0
            self.get_logger().info(f'🚀 Moving Forward | Distance: {distances[0]:.2f} m | Speed: {speed:.2f}')
        
        elif distances[180] < 4.0:  # วัตถุด้านหลัง -> ถอยหลัง
            self.twist.linear.x = -speed
            self.twist.angular.z = 0.0
            self.get_logger().info(f'🔄 Moving Backward | Distance: {distances[180]:.2f} m | Speed: {speed:.2f}')
        
        elif distances[270] < 4.0:  # วัตถุด้านขวา -> หมุน CW
            self.twist.linear.x = 0.0
            self.twist.angular.z = -speed
            self.get_logger().info(f'🔁 Rotating CW | Distance: {distances[270]:.2f} m | Speed: {speed:.2f}')
        
        elif distances[90] < 4.0:  # วัตถุด้านซ้าย -> หมุน CCW
            self.twist.linear.x = 0.0
            self.twist.angular.z = speed
            self.get_logger().info(f'↩ Rotating CCW | Distance: {distances[90]:.2f} m | Speed: {speed:.2f}')
        
        else:  # ไม่มีวัตถุใกล้ -> หยุด
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            self.get_logger().info(f'🛑 Stopping | No Object Nearby')

        # ส่งคำสั่งควบคุม
        self.publisher.publish(self.twist)
        self.get_logger().info(f'Published cmd_vel: linear.x={self.twist.linear.x:.2f}, angular.z={self.twist.angular.z:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = TurtleBotController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.twist.linear.x = 0.0
        node.twist.angular.z = 0.0
        node.publisher.publish(node.twist)
        node.get_logger().info("🛑 CTRL+C Pressed! Stopping TurtleBot3.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
