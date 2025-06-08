import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Int32
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Quaternion # Import Quaternion for orientation
import tf_transformations

import math
import serial
import time
class AckermannSerialController(Node):
    def __init__(self):
        super().__init__('ackermann_serial_controller')
        
        # Robot parameters
        self.wheelbase = 0.225              # meters
        self.wheel_radius = 0.125 * 0.5     # meters
        self.serial_port = '/dev/ttyACM1'
        self.baudrate = 1000000

        self.last_lat = None
        self.last_lon = None
        self.last_heading_rad = None
        
        # Initialize serial
        try:
            self.ser = serial.Serial(self.serial_port, self.baudrate, timeout=1)
            self.get_logger().info(f"Serial connected on {self.serial_port}")
        except Exception as e:
            self.get_logger().error(f"Serial connection failed: {e}")
            self.ser = None
        
        # Subscribe to cmd_vel (original functionality)
        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Subscribe to individual steering and speed topics
        self.steer_subscription = self.create_subscription(
            Float32,
            'steer',
            self.steer_callback,
            10
        )
        
        self.speed_subscription = self.create_subscription(
            Int32,
            'speed',
            self.speed_callback,
            10
        )
        
        self.gps_publisher = self.create_publisher(NavSatFix, 'gps/fix', 10)
        self.heading_imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        self.pose2d_pub = self.create_publisher(Pose2D, 'gps/pose2d', 10)


        self.serial_timer = self.create_timer(0.005, self.read_serial_data)  # 20 Hz
        
        
        self.get_logger().info("Subscribed to topics: cmd_vel, steer, speed")

    def cmd_vel_callback(self, msg):
        """Original cmd_vel callback - converts Twist to steering and motor commands"""
        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z
        
        # Compute steering angle in radians
        if abs(linear_velocity) < 1e-4:
            steering_angle_rad = 0.0
        elif angular_velocity == 0.0:
            steering_angle_rad = 0.0
        else:
            steering_angle_rad = math.atan2(angular_velocity * self.wheelbase, linear_velocity)
        
        # Convert to degrees and map to servo range
        steering_angle_deg = math.degrees(steering_angle_rad)
        servo_angle = 90 + int(steering_angle_deg)
        servo_angle = max(45, min(135, servo_angle))  # Clamp to [45, 135]
        
        # Compute RPM
        rpm = int((linear_velocity / (2 * math.pi * self.wheel_radius)) * 60)
        
        # Send both commands without delay
        self.send_steering_command(servo_angle)
        self.send_motor_command(rpm)

    def steer_callback(self, msg):
        """Direct steering angle control (in degrees, 0 = center)"""
        steering_angle = msg.data
        # Convert to servo range: 0 degrees input -> 90 degrees servo (center)
        servo_angle = 90 + int(steering_angle)
        # Clamp to safe servo range (45-135 degrees)
        servo_angle = max(45, min(135, servo_angle))
        self.send_steering_command(servo_angle)

    def speed_callback(self, msg):
        """Direct speed control (RPM)"""
        rpm = msg.data
        self.send_motor_command(rpm)

    def send_steering_command(self, servo_angle):
        """Send steering command via serial"""
        if self.ser and self.ser.is_open:
            try:
                steer_cmd = f'SERV{servo_angle}\n'
                self.ser.write(steer_cmd.encode())
                self.get_logger().info(f"Sent steering: {steer_cmd.strip()}")
                time.sleep(0.05)
            except Exception as e:
                self.get_logger().error(f"Steering command failed: {e}")
        else:
            self.get_logger().warn("Serial not connected - steering command ignored")

    def send_motor_command(self, rpm):
        """Send motor command via serial"""
        if self.ser and self.ser.is_open:
            try:
                motor_cmd = f'MOTT{rpm}\n'
                self.ser.write(motor_cmd.encode())
                time.sleep(0.05)
                self.get_logger().info(f"Sent motor: {motor_cmd.strip()}")
            except Exception as e:
                self.get_logger().error(f"Motor command failed: {e}")
        else:
            self.get_logger().warn("Serial not connected - motor command ignored")

    def read_serial_data(self):
        if self.ser and self.ser.in_waiting:
            try:
                line = self.ser.readline().decode().strip()
                if line.startswith("GPS"):
                    gps_data = line[3:]  # Remove "GPS"
                    parts = gps_data.split(',')
                    if len(parts) == 3:
                        lat = float(parts[0])
                        lon = float(parts[1])
                        speed = float(parts[2])  # Optional: not used here

                        self.last_lat = lat
                        self.last_lon = lon


                        gps_msg = NavSatFix()
                        gps_msg.latitude = lat
                        gps_msg.longitude = lon
                        gps_msg.altitude = 0.0  # Dummy altitude unless you have it

                        gps_msg.header.stamp = self.get_clock().now().to_msg()
                        gps_msg.header.frame_id = "base_link"  # Use appropriate frame

                        gps_msg.status.status = 0  # STATUS_FIX
                        gps_msg.status.service = 1  # SERVICE_GPS

                        self.gps_publisher.publish(gps_msg)
                        #self.get_logger().info(f"Published GPS: {lat}, {lon}")

                if line.startswith("CMP"):
                    heading_str = line[3:]  # Remove "CMP"
                    try:
                        heading_deg = float(heading_str)
                        heading_rad = math.radians(heading_deg)
                        self.last_heading_rad = heading_rad
                        # Create an Imu message
                        imu_msg = Imu()

                        # Set the timestamp (important for sensor data)
                        imu_msg.header.stamp = self.get_clock().now().to_msg()
                        imu_msg.header.frame_id = 'base_link' # Or your appropriate frame_id

                        # Convert yaw (heading_rad) to a quaternion
                        # This assumes your heading is yaw around the Z-axis
                        quaternion = tf_transformations.quaternion_from_euler(0, 0, heading_rad)
                        imu_msg.orientation.x = quaternion[0]
                        imu_msg.orientation.y = quaternion[1]
                        imu_msg.orientation.z = quaternion[2]
                        imu_msg.orientation.w = quaternion[3]

                      
                        imu_msg.orientation_covariance[0] = -1.0 # Indicate unknown covariance

                        self.heading_imu_pub.publish(imu_msg)
                        #self.get_logger().info(f"Published IMU heading (rad): {heading_rad:.3f}")

                        if self.last_lat is not None and self.last_lon is not None:
                            pose_msg = Pose2D()
                            pose_msg.x = self.last_lat
                            pose_msg.y = self.last_lon
                            pose_msg.theta = heading_rad
                            self.pose2d_pub.publish(pose_msg)
                            #self.get_logger().info(f"Published Pose2D: x={self.last_lat}, y={self.last_lon}, theta={heading_rad:.3f}")

                    except ValueError:
                        self.get_logger().warn(f"Invalid heading format: {heading_str}")


            except Exception as e:
                self.get_logger().error(f"GPS parse error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = AckermannSerialController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser and node.ser.is_open:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()