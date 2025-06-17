import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Int32
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Quaternion # Import Quaternion for orientation
import tf_transformations
import tf2_ros # Import for TF broadcasting
from nav_msgs.msg import Odometry # Import Odometry message type


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

        # Odometry variables
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_theta = 0.0
        self.last_odom_time = self.get_clock().now()
        self.current_angular_vel_cmd = 0.0 #

        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10) # Odometry publisher
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self) # TF broadcaster
        self.steering_angle = 0.0
        
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
        self.steering_angle = servo_angle - 90
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

                elif line.startswith("CMP"):
                    
                    try:
                        # Remove "CMP" prefix and split the string by commas
                        data_str = line[3:].strip()
                        values = data_str.split(',')

                        # Ensure we have exactly 7 values (gx, gy, gz, ax, ay, az, heading)
                        if len(values) == 7:
                            gx = float(values[0])
                            gy = float(values[1])
                            gz = float(values[2])
                            ax = float(values[3])
                            ay = float(values[4])
                            az = float(values[5])
                            heading_deg = float(values[6])

                            heading_rad = math.radians(heading_deg)
                            self.last_heading_rad = heading_rad # Store if needed elsewhere

                            # Create an Imu message
                            imu_msg = Imu()

                            # Set the timestamp (important for sensor data)
                            imu_msg.header.stamp = self.get_clock().now().to_msg()
                            imu_msg.header.frame_id = 'base_link' # Or your appropriate frame_id

                            # Populate angular velocity (gyroscope data)
                            imu_msg.angular_velocity.x = gx
                            imu_msg.angular_velocity.y = gy
                            imu_msg.angular_velocity.z = gz

                            # Populate linear acceleration (accelerometer data)
                            imu_msg.linear_acceleration.x = ax
                            imu_msg.linear_acceleration.y = ay
                            imu_msg.linear_acceleration.z = az

                            # Convert yaw (heading_rad) to a quaternion
                            # This assumes your heading is yaw around the Z-axis
                            # Ensure your coordinate systems align:
                            # ROS convention: X-forward, Y-left, Z-up
                            # If your heading is clockwise from North, you might need to adjust the sign.
                            # Assuming heading_rad is counter-clockwise from X-axis (East), or North (Y-axis) in a typical setup.
                            # Adjust Euler angles if your sensor's axes are different.
                            quaternion = tf_transformations.quaternion_from_euler(0, 0, heading_rad)
                            imu_msg.orientation.x = quaternion[0]
                            imu_msg.orientation.y = quaternion[1]
                            imu_msg.orientation.z = quaternion[2]
                            imu_msg.orientation.w = quaternion[3]

                            # Set covariance for orientation, angular velocity, and linear acceleration
                            # -1.0 indicates "unknown/unspecified" covariance.
                            # If you have sensor datasheets, you can provide more accurate values.
                            imu_msg.orientation_covariance[0] = -1.0 # For x
                            imu_msg.orientation_covariance[4] = -1.0 # For y
                            imu_msg.orientation_covariance[8] = 0.01 # For z

                            imu_msg.angular_velocity_covariance[0] = -1.0 # For x
                            imu_msg.angular_velocity_covariance[4] = -1.0 # For y
                            imu_msg.angular_velocity_covariance[8] = 0.0001 #or z

                            imu_msg.linear_acceleration_covariance[0] = -1.0 # For x
                            imu_msg.linear_acceleration_covariance[4] = -1.0 # For y
                            imu_msg.linear_acceleration_covariance[8] = -1.0 # For z

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

                elif line.startswith("KIN"):
                    try:
                        # Parse the linear velocity reported by the Arduino
                        reported_linear_vel = float(line[3:])

                        current_time = self.get_clock().now()
                        dt = (current_time - self.last_odom_time).nanoseconds / 1e9 # Convert to seconds
                        self.last_odom_time = current_time

                        # If dt is very small or zero, skip to avoid division by zero
                        if dt < 1e-6:
                            return

                        # The linear velocity is directly from the Arduino
                        vx = reported_linear_vel
                        
                        A = math.tan(self.steering_angle*0.5/180*3.14159)
                        if A != 0:
                            R = self.wheelbase/A
                            wz = vx/R
                        else:
                            wz = 0.0
                        
                        

                      
                        delta_x = vx * math.cos(self.odom_theta) * dt
                        delta_y = vx * math.sin(self.odom_theta) * dt
                        delta_theta = wz * dt

                        self.odom_x += delta_x
                        self.odom_y += delta_y
                        self.odom_theta += delta_theta

                        # Normalize theta to be between -pi and pi
                        self.odom_theta = math.atan2(math.sin(self.odom_theta), math.cos(self.odom_theta))


                        # --- Publish Odometry Message ---
                        odom_msg = Odometry()
                        odom_msg.header.stamp = current_time.to_msg()
                        odom_msg.header.frame_id = 'odom' # The fixed frame
                        odom_msg.child_frame_id = 'base_link' # The robot's base frame

                        # Set the position
                        odom_msg.pose.pose.position.x = self.odom_x
                        odom_msg.pose.pose.position.y = self.odom_y
                        odom_msg.pose.pose.position.z = 0.0 # 2D robot

                        # Set the orientation (from current theta)
                        q = tf_transformations.quaternion_from_euler(0, 0, self.odom_theta)
                        odom_msg.pose.pose.orientation.x = q[0]
                        odom_msg.pose.pose.orientation.y = q[1]
                        odom_msg.pose.pose.orientation.z = q[2]
                        odom_msg.pose.pose.orientation.w = q[3]

                        # Set the velocities
                        odom_msg.twist.twist.linear.x = vx
                        odom_msg.twist.twist.linear.y = 0.0
                        odom_msg.twist.twist.angular.z = wz

                        # Set covariance (optional, but good practice; use -1 if unknown)
                        # A very basic guess for covariance. In a real system, these would be tuned.
                        odom_msg.pose.covariance[0] = 0.01  # x
                        odom_msg.pose.covariance[7] = 0.01  # y
                        odom_msg.pose.covariance[35] = 0.02 # theta (yaw)
                        odom_msg.twist.covariance[0] = 0.01 # vx
                        odom_msg.twist.covariance[35] = 0.02 # wz

                        self.odom_publisher.publish(odom_msg)
                        # self.get_logger().info(f"Published Odometry: x={self.odom_x:.2f}, y={self.odom_y:.2f}, theta={self.odom_theta:.2f}, vx={vx:.2f}, wz={wz:.2f}")


                        # --- Publish TF Transform (odom -> base_link) ---
                        t = tf2_ros.TransformStamped()
                        t.header.stamp = current_time.to_msg()
                        t.header.frame_id = 'odom'
                        t.child_frame_id = 'base_link'

                        t.transform.translation.x = self.odom_x
                        t.transform.translation.y = self.odom_y
                        t.transform.translation.z = 0.0
                        t.transform.rotation.x = q[0]
                        t.transform.rotation.y = q[1]
                        t.transform.rotation.z = q[2]
                        t.transform.rotation.w = q[3]

                        #self.tf_broadcaster.sendTransform(t)

                    except ValueError:
                        self.get_logger().warn(f"Invalid KIN format: {line[3:]}")
                    except IndexError:
                        self.get_logger().warn(f"Incomplete KIN data received: {line}")

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