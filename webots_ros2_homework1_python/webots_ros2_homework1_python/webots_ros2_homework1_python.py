import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from rclpy.qos import ReliabilityPolicy, QoSProfile
import math

LINEAR_VEL = 0.22
STOP_DISTANCE = 0.2
LIDAR_ERROR = 0.05
LIDAR_AVOID_DISTANCE = 0.7
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR
RIGHT_SIDE_INDEX = 270
RIGHT_FRONT_INDEX = 210
LEFT_FRONT_INDEX=150
LEFT_SIDE_INDEX=90
MAX_SPEED = 3

DESIRED_DISTANCE = 1
TEST_NAME = "1_meter_sim"

class RandomWalk(Node):

    def __init__(self):
        # Initialize the publisher
        super().__init__('random_walk_node')
        self.scan_cleaned = []
        self.stall = False
        self.turtlebot_moving = False
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscriber1 = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback1,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.subscriber2 = self.create_subscription(
            Odometry,
            '/odom',
            self.listener_callback2,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.laser_forward = 0
        self.odom_data = 0
        self.pose_saved = ''
        self.cmd = Twist()
        self.timer = self.create_timer(0.5, self.timer_callback)

        self.odom_log_file = open('{TEST_NAME}.txt', 'w')

    def listener_callback1(self, msg1):
        scan = msg1.ranges
        self.scan_cleaned = []
        for reading in scan:
            if reading == float('Inf'):
                self.scan_cleaned.append(3.5)
            elif math.isnan(reading):
                self.scan_cleaned.append(0.0)
            else:
                self.scan_cleaned.append(reading)

    def listener_callback2(self, msg2):
        position = msg2.pose.pose.position
        orientation = msg2.pose.pose.orientation
        (posx, posy, posz) = (position.x, position.y, position.z)
        (qx, qy, qz, qw) = (orientation.x, orientation.y, orientation.z, orientation.w)
        self.get_logger().info('self position: {},{},{}'.format(posx, posy, posz))
        self.pose_saved = position

        self.odom_log_file.write(f"Position: x={posx}, y={posy}, z={posz}, Orientation: qx={qx}, qy={qy}, qz={qz}, qw={qw}\n")

    def timer_callback(self):
        if len(self.scan_cleaned) == 0:
            self.turtlebot_moving = False
            return
        
        front_lidar_min = min(self.scan_cleaned[LEFT_FRONT_INDEX:RIGHT_FRONT_INDEX])

        if front_lidar_min < SAFE_STOP_DISTANCE:
            self.cmd.linear.x = 0.0 
            self.cmd.angular.z = 0.0 
            self.publisher_.publish(self.cmd)
            self.turtlebot_moving = False
            self.get_logger().info('Stopping')
            return
        else:
            distance = self.pose_saved.x
            if distance < DESIRED_DISTANCE:
                if distance < DESIRED_DISTANCE - STOP_DISTANCE:
                    speed = MAX_SPEED
                else:
                    speed = max(0.5, MAX_SPEED * (distance - DESIRED_DISTANCE) / STOP_DISTANCE)
                    self.set_speed(speed)
            else:
                self.set_speed(0)

    def set_speed(self, speed):
        self.cmd.linear.x = speed
        self.cmd.angular.z = 0.0
        self.publisher_.publish(self.cmd)

    def destroy_node(self):
        # Close the log file when the node is destroyed
        self.odom_log_file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    random_walk_node = RandomWalk()
    rclpy.spin(random_walk_node)
    random_walk_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
