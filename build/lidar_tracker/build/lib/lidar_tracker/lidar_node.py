import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class LidarTracker(Node):
    def __init__(self):
        super().__init__('lidar_tracker')
        
        # Subscriber สำหรับรับค่าจาก Lidar
        self.subscription = self.create_subscription(
            LaserScan, 
            '/scan', 
            self.lidar_callback, 
            10)

        # Publisher สำหรับส่งคำสั่งความเร็วไปที่ TurtleBot3
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.twist = Twist()

    def lidar_callback(self, msg):
        ranges = msg.ranges  # ข้อมูลระยะของวัตถุรอบตัวหุ่นยนต์
        angles = [0, 90, 180, 270]  # องศาหลัก
        tolerance = 30  # องศาที่ใช้พิจารณาวัตถุ

        # ตรวจสอบค่าระยะที่ผิดพลาด (เช่น inf) และแทนด้วย 4.0 เมตร (ไม่มีวัตถุ)
        valid_ranges = [r if r > 0 else 4.0 for r in ranges]

        # หาค่าระยะที่ใกล้ที่สุดในแต่ละทิศ
        distances = {
            angle: min(valid_ranges[max(0, angle - tolerance): min(len(ranges), angle + tolerance + 1)])
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
        
        elif distances[270] < 4.0:  # วัตถุด้านขวา -> หมุนตามเข็ม (CW)
            self.twist.linear.x = 0.0
            self.twist.angular.z = -speed
            self.get_logger().info(f'🔁 Rotating CW | Distance: {distances[270]:.2f} m | Speed: {speed:.2f}')
        
        elif distances[90] < 4.0:  # วัตถุด้านซ้าย -> หมุนทวนเข็ม (CCW)
            self.twist.linear.x = 0.0
            self.twist.angular.z = speed
            self.get_logger().info(f'↩ Rotating CCW | Distance: {distances[90]:.2f} m | Speed: {speed:.2f}')
        
        else:  # ไม่มีวัตถุ -> หยุด
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            self.get_logger().info(f'🛑 Stopping | No Object Nearby')

        # ส่งคำสั่งไปที่หุ่นยนต์
        self.publisher.publish(self.twist)

def main(args=None):
    rclpy.init(args=args)
    node = LidarTracker()
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
