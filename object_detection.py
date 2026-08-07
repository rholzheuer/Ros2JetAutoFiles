import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy


class ObjectDetection(Node):

    def __init__(self):
        super().__init__('object_detection')

        # BEST EFFORT QoS for unstable network
        qos_profile = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

        # publisher for velocity
        self.publisher_= self.create_publisher(Twist, '/controller/cmd_vel', 10)

        # subsciber for LiDAR data
        self.subscription = self.create_subscription(
            LaserScan, 
            '/scan', 
            self.lidar_callback, 
            qos_profile)
 
        self.subscription   # prevent unused variable warning

        # Twist Object with forward velocity
        self.msg = Twist()          
        self.msg.linear.x = 0.1     # [m/s]    

        self.safe_distance = 0.5    # [m]

        self.timer = self.create_timer(0.1, self.publish_velocity)

        self.get_logger().info('ObjectDetection node started')


    def publish_velocity(self):
        self.publisher_.publish(self.msg)


    def lidar_callback(self, msg):
        
        # filter out invalid LiDAR data (0 or beyond max range)
        valid_ranges = [r for r in msg.ranges if r > 0.0 and r < msg.range_max]
        if not valid_ranges:
            return
        
        min_distance = min(valid_ranges)

        if min_distance <= self.safe_distance:
            self.msg.linear.x = 0.0
        else:
            self.msg.linear.x = 0.1


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetection()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()