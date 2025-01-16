import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class GPSNavigator(Node):
    def __init__(self):
        super().__init__('gps_navigator')

        # Subscribers
        self.create_subscription(NavSatFix, '/gps/goal', self.goal_callback, 10)
        self.create_subscription(NavSatFix, '/fix', self.gps_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)

        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd/vel', 10)

        # Current States
        self.current_gps = None  # Stores current position
        self.goal_gps = None      # Stores goal position
        self.current_yaw = None   # Yaw from IMU
        self.current_velocity = None  # From Odometry

        # Constants
        self.speed = 0.5  # m/s (adjustable)
        self.angle_tolerance = 5  # Degrees, allowable heading error
        self.distance_tolerance = 0.5  # Meters

    def goal_callback(self, msg: NavSatFix):
        """
        Receives a goal GPS position and stores it.
        Input -> msg: NavSatFix
        Output -> Updates self.goal_gps with the received goal position.
        """
        self.goal_gps = msg
        self.get_logger().info(f'Received goal GPS: Lat {msg.latitude}, Lon {msg.longitude}')

    def gps_callback(self, msg: NavSatFix):
        """
        Receives the current GPS position and stores it.
        Input -> msg: NavSatFix
        Output -> Updates self.current_gps with the received current GPS position.
        """
        self.current_gps = msg

    def odom_callback(self, msg: Odometry):
        """
        Receives velocity data from odometry and stores it.
        Input -> msg: Odometry
        Output -> Updates self.current_velocity with the received velocity data.
        """
        self.current_velocity = msg.twist.twist.linear.x

    def imu_callback(self, msg: Imu):
        """
        Receives yaw orientation from IMU and converts it to yaw.
        Input -> msg: Imu
        Output -> Updates self.current_yaw with the converted yaw value.
        """
        self.current_yaw = self.quaternion_to_yaw(msg.orientation)

    """
     Yaw Angle Representation
    -------------------------------------------
     0° (or 360°) → Facing North.
     90° → Facing East.
     180° → Facing South.
     270° → Facing West.
     Positive yaw → Counterclockwise rotation.
     Negative yaw → Clockwise rotation.
     -----------------------------------------
    """
    def quaternion_to_yaw(self, orientation):
        """
        Converts quaternion IMU orientation to yaw angle in degrees.
        Input -> orientation: geometry_msgs/Quaternion
        Output -> Returns the yaw angle in degrees.
        """
        import tf_transformations
        quaternion = (
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        )
        _, _, yaw = tf_transformations.euler_from_quaternion(quaternion)
        return math.degrees(yaw)    
    
    def haversine_distance(self, lat1, lon1, lat2, lon2):
        """ 
        Calculates great-circle distance between two GPS coordinates using the Haversine formula. 
        Input -> lat1, lon1, lat2, lon2 (float): Latitude and longitude of two points.
        Output -> Returns the distance in meters (float).
        """

        R = 6371000  # Radius of the Earth in meters

        # Convert latitude and longitude from degrees to radians
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        delta_phi = math.radians(lat2 - lat1)
        delta_lambda = math.radians(lon2 - lon1)

        # Haversine formula
        a = math.sin(delta_phi / 2.0) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2.0) ** 2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

        # Compute distance
        distance = R * c

        return distance

    def calculate_bearing(self, lat1, lon1, lat2, lon2):
        """
        Computes the heading angle (bearing) from the current position to the goal.
        Input -> lat1, lon1, lat2, lon2 (float): Latitude and longitude of current and goal position.
        Output -> Returns the bearing angle in degrees (float).
        """
        
        # Convert latitude and longitude from degrees to radians
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        delta_lambda = math.radians(lon2 - lon1)

        # Calculate bearing
        y = math.sin(delta_lambda) * math.cos(phi2)
        x = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(delta_lambda)
        
        # Convert radians to degrees and normalize to [0, 360]
        bearing = (math.degrees(math.atan2(y, x)) + 360) % 360

        return bearing
    

        """ 
        ----------------------------------------------------------------------
        |   Step   |       Operation                                         |
        |--------------------------------------------------------------------|
        | Check     |   If GPS & IMU data are available                      |
        | Compute   |   Distance to goal                                     |
        | Stop	    |   If distance < tolerance                              |
        | Compute   |   Desired heading using calculate_bearing              | 
        | Compute   |   Heading error = Desired heading - Current yaw        |
        | Normalize |   Heading error to [-180, 180]                         |
        | Decide    |   Rotate if error > angle tolerance, else move forward |
        ----------------------------------------------------------------------
        """
    def compute_navigation(self):
        """
        Computes movement commands based on GPS goal, heading, and position.
        Input -> None.
        Output -> Publishes velocity commands to adjust the rover's motion.
        """

        # Ensure we have valid GPS and IMU data
        if self.current_gps is None or self.goal_gps is None or self.current_yaw is None:
            print("Waiting for GPS, goal, and IMU data...")
            return

        # Extract latitude and longitude
        lat1, lon1 = self.current_gps.latitude, self.current_gps.longitude
        lat2, lon2 = self.goal_gps.latitude, self.goal_gps.longitude

        # Compute distance to goal
        distance = self.haversine_distance(lat1, lon1, lat2, lon2)
        print(f"Distance to goal: {distance:.2f} meters")

        # Stop if the goal is reached
        if distance < self.distance_tolerance:
            print("Goal reached! Stopping.")
            self.publish_velocity(0.0, 0.0)
            return

        # Compute desired heading
        desired_heading = self.calculate_bearing(lat1, lon1, lat2, lon2)
        heading_error = desired_heading - self.current_yaw

        # Normalize heading error to [-180, 180] range
        heading_error = (heading_error + 180) % 360 - 180

        print(f"Desired heading: {desired_heading:.2f}°, Heading error: {heading_error:.2f}°")

        # Decide whether to rotate or move forward
        if abs(heading_error) > self.angle_tolerance:
            # Rotate in place
            angular_speed = 0.3 if heading_error > 0 else -0.3
            print(f"Rotating at {angular_speed} rad/s")
            self.publish_velocity(0.0, angular_speed)
        else:
            # Move forward
            print(f"Moving forward at {self.speed} m/s")
            self.publish_velocity(self.speed, 0.0)


    def publish_velocity(self, linear_x, angular_z):
        """
        Publishes velocity commands to the /cmd/vel topic.
        Input -> linear_x (float): Linear velocity in m/s.
                angular_z (float): Angular velocity in rad/s.
        Output -> Publishes a Twist message containing velocity commands to control rover movement.
        """

        # Create a Twist message
        cmd_msg = Twist()
        
        # Set linear and angular velocity
        cmd_msg.linear.x = linear_x
        cmd_msg.angular.z = angular_z

        # Publish to /cmd/vel
        self.cmd_vel_pub.publish(cmd_msg)
        
        print(f"Publishing velocity - Linear: {linear_x} m/s, Angular: {angular_z} rad/s")

def main(args=None):
    rclpy.init(args=args)
    navigator = GPSNavigator()
    rclpy.spin(navigator)
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
