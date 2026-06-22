import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Range, Imu
from geometry_msgs.msg import Twist
import math

class BehaviorNode(Node):
    def __init__(self):
        super().__init__('behavior_node')
        
        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribers
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.range_sub = self.create_subscription(Range, '/ultrasound_front', self.range_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        
        # State
        self.lidar_clear = True
        self.ultrasonic_clear = True
        self.imu_ok = True
        self.forward_speed = 0.3
        
        # Timer - publish cmd_vel at 10Hz
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info('Behavior node started')

    def scan_callback(self, msg):
        # Check front of robot (-30 to +30 degrees)
        ranges = msg.ranges
        total = len(ranges)
        front_start = int(total * 0.0)
        front_end   = int(total * 0.08)  # ~30 degrees
        back_start  = int(total * 0.92)

        front_ranges = ranges[front_start:front_end] + ranges[back_start:]
        valid = [r for r in front_ranges if not math.isinf(r) and r > 0.1]

        if valid and min(valid) < 0.5:
            self.lidar_clear = False
            self.get_logger().warn('LiDAR: obstacle ahead at ' + str(round(min(valid), 2)) + 'm')
        else:
            self.lidar_clear = True

    def range_callback(self, msg):
        if msg.range < 0.30 and msg.range > 0.02:
            self.ultrasonic_clear = False
            self.get_logger().warn('Ultrasonic: obstacle at ' + str(round(msg.range, 2)) + 'm')
        else:
            self.ultrasonic_clear = True

    def imu_callback(self, msg):
        roll  = msg.linear_acceleration.y
        pitch = msg.linear_acceleration.x
        if abs(roll) > 8.0 or abs(pitch) > 8.0:
            self.imu_ok = False
            self.get_logger().warn('IMU: tilt detected!')
        else:
            self.imu_ok = True

    def control_loop(self):
        cmd = Twist()
        if self.lidar_clear and self.ultrasonic_clear and self.imu_ok:
            cmd.linear.x = self.forward_speed
            cmd.angular.z = 0.0
        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = BehaviorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
