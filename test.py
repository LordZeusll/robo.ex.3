import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from nav_msgs.msg import Odometry
import math
import time


class GoToPose(Node):

    def __init__(self):
        super().__init__('GoToPoseCmd_publisher')

        # Initialize publishers and subscribers
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.coord_publisher_ = self.create_publisher(String, '/coordinates', 10)
        self.odom_subscriber_ = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.new_target_subscriber_ = self.create_subscription(String, '/new_target_pose', self.new_target_callback, 10)

        self.timer = self.create_timer(0.05, self.timer_callback)

        # Initialize target pose (x, y, theta)
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_theta = 0.0
        self.state = "idle"

        # Current pose
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        # Tolerances
        self.position_tolerance = 0.01  # 5 cm
        self.angle_tolerance = 0.01  # ~5.7 degrees

        # Internal navigation step
        self.navigation_step = 0

        # Shape movement tracking
        self.square_step = 0
        self.circle_start_yaw = None

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        # Convert quaternion to yaw angle
        orientation_q = msg.pose.pose.orientation
        _, _, self.current_yaw = self.quaternion_to_euler(orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)

    def quaternion_to_euler(self, x, y, z, w):
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return 0.0, 0.0, yaw

    def new_target_callback(self, msg):
        data = msg.data.lower().strip()

        # Immediately stop any ongoing action
        self.state = "idle"
        self.stop_turtlebot()
        time.sleep(0.1)

        # Parse coordinate command
        if "," in data:
            try:
                parts = data.split(",")
                self.target_x = float(parts[0])
                self.target_y = float(parts[1])
                self.target_theta = math.radians(float(parts[2]))
                self.state = "navigate"
                self.navigation_step = 0
                self.get_logger().info(f"New target received: x={self.target_x}, y={self.target_y}, theta={math.degrees(self.target_theta):.2f}")
            except ValueError:
                self.get_logger().error("Invalid coordinate format. Expected 'x,y,theta'.")
        elif data == "circle":
            self.state = "circle"
            self.circle_start_yaw = self.current_yaw
            self.get_logger().info("Circle command received. Performing one full circular rotation.")
        elif data == "square":
            self.state = "square"
            self.square_step = 0
            self.get_logger().info("Square command received. Moving in a square path.")
        else:
            self.get_logger().info(f"Unknown command received: {data}")

    def timer_callback(self):
        move_cmd = Twist()

        if self.state == "navigate":
            self.navigate_to_target(move_cmd)
        elif self.state == "circle":
            self.perform_circle(move_cmd)
        elif self.state == "square":
            self.perform_square(move_cmd)
        elif self.state == "idle":
            move_cmd.linear.x = 0.0
            move_cmd.angular.z = 0.0
            self.get_logger().info("Robot is idle, waiting for new commands.")
            self.publisher_.publish(move_cmd)

        self.publish_coordinates()

    def navigate_to_target(self, move_cmd):
        if self.navigation_step == 0:
            # Step 1: Rotate to face the target x
            target_angle_to_x = math.atan2(0, self.target_x - self.current_x)  # Target x only
            angle_diff = self.normalize_angle(target_angle_to_x - self.current_yaw)

            if abs(angle_diff) > self.angle_tolerance:
                move_cmd.angular.z = 0.3 if angle_diff > 0 else -0.3
            else:
                self.navigation_step = 1
                self.stop_turtlebot()
                self.get_logger().info(f"Aligned to face x, moving towards x = {self.target_x:.2f}")

        elif self.navigation_step == 1:
            # Step 2: Move towards the target x (keep y fixed)
            distance_x = self.target_x - self.current_x
            if abs(distance_x) > self.position_tolerance:
                move_cmd.linear.x = 0.2
            else:
                self.navigation_step = 2
                self.stop_turtlebot()
                self.get_logger().info(f"Reached x = {self.target_x:.2f}")

        elif self.navigation_step == 2:
            # Step 3: Rotate to face the target y (move along y-axis)
            target_angle_to_y = math.atan2(self.target_y - self.current_y, self.target_x - self.current_x)
            angle_diff = self.normalize_angle(target_angle_to_y - self.current_yaw)

            if abs(angle_diff) > self.angle_tolerance:
                move_cmd.angular.z = 0.3 if angle_diff > 0 else -0.3
            else:
                self.navigation_step = 3
                self.stop_turtlebot()
                self.get_logger().info(f"Aligned to face y, moving towards y = {self.target_y:.2f}")

        elif self.navigation_step == 3:
            # Step 4: Move towards the target y (keep x fixed)
            distance_y = self.target_y - self.current_y
            if abs(distance_y) > self.position_tolerance:
                move_cmd.linear.x = 0.2
            else:
                self.navigation_step = 4
                self.stop_turtlebot()
                self.get_logger().info(f"Reached y = {self.target_y:.2f}")

        elif self.navigation_step == 4:
            # Step 5: Rotate to face target_theta
            angle_diff = self.normalize_angle(self.target_theta - self.current_yaw)
            if abs(angle_diff) > self.angle_tolerance:
                move_cmd.angular.z = 0.3 if angle_diff > 0 else -0.3
            else:
                self.state = "idle"
                self.stop_turtlebot()
                self.get_logger().info(f"Reached target orientation (theta = {math.degrees(self.target_theta):.2f}).")

        self.publisher_.publish(move_cmd)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def perform_circle(self, move_cmd):
        if self.circle_start_yaw is None:
            self.circle_start_yaw = self.current_yaw

        # Calculate the absolute yaw difference
        yaw_diff = self.normalize_angle(self.current_yaw - self.circle_start_yaw)

        if abs(yaw_diff) < 2 * math.pi:  # Less than one full rotation
            move_cmd.linear.x = 0.2
            move_cmd.angular.z = 0.5
        else:
            self.state = "idle"
            self.circle_start_yaw = None  # Reset for future circles
            self.stop_turtlebot()
            self.get_logger().info("Completed one full circular rotation.")

        self.publisher_.publish(move_cmd)

    def perform_square(self, move_cmd):
        if self.square_step < 4:  # 4 sides of the square
            # Move forward for the side length
            move_cmd.linear.x = 0.2
            self.publisher_.publish(move_cmd)
            time.sleep(1.0)  # Assume each side takes 1 second at 0.2 m/s

            # Stop briefly before turning
            self.stop_turtlebot()
            time.sleep(0.5)

            # Rotate 90 degrees
            move_cmd.angular.z = math.radians(90)
            self.publisher_.publish(move_cmd)
            time.sleep(1.0)  # Rotate 90 degrees

            # Stop after turning
            self.stop_turtlebot()
            self.square_step += 1
        else:
            self.state = "idle"
            self.get_logger().info("Completed square path.")

    def publish_coordinates(self):
        coord_msg = String()
        coord_msg.data = f"Coordinates: x={self.current_x:.2f}, y={self.current_y:.2f}, theta={math.degrees(self.current_yaw):.2f}"
        self.coord_publisher_.publish(coord_msg)
        self.get_logger().info(coord_msg.data)

    def stop_turtlebot(self):
        self.publisher_.publish(Twist())
        time.sleep(0.1)  # Short delay to ensure the robot stops


def main(args=None):
    rclpy.init(args=args)
    node = GoToPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_turtlebot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

