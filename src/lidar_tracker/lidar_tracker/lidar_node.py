import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LidarTracker(Node):
    def __init__(self):
        super().__init__('lidar_tracker')
        self.subscription = self.create_subscription(
            LaserScan, 
            '/scan', 
            self.lidar_callback, 
            10)
        self.subscription

    def lidar_callback(self, msg):
        ranges = msg.ranges  # ข้อมูลระยะของวัตถุรอบตัวหุ่นยนต์
        min_distance = min(ranges)  # ค่าที่ใกล้ที่สุด
        angle = ranges.index(min_distance)  # หามุมที่ระยะน้อยสุด

        self.get_logger().info(f'Distance: {min_distance:.2f} m, Angle: {angle}°')

def main(args=None):
    rclpy.init(args=args)
    node = LidarTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
