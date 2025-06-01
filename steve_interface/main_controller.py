import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import serial
import time  # <-- Added for delay

class AckermannSerialController(Node):
    def __init__(self):
        super().__init__('ackermann_serial_controller')

        # Robot parameters
        self.wheelbase = 0.225              # meters
        self.wheel_radius = 0.125 * 0.5     # meters
        self.serial_port = '/dev/ttyACM1'
        self.baudrate = 115200

        # Initialize serial
        try:
            self.ser = serial.Serial(self.serial_port, self.baudrate, timeout=1)
            self.get_logger().info(f"Serial connected on {self.serial_port}")
        except Exception as e:
            self.get_logger().error(f"Serial connection failed: {e}")
            self.ser = None

        # Subscribe to cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

    def cmd_vel_callback(self, msg):
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
        servo_angle = max(45, min(135, servo_angle))  # Clamp to [60, 120]

        # Compute RPM
        rpm = int((linear_velocity / (2 * math.pi * self.wheel_radius)) * 60)

        # Send commands with delay
        if self.ser and self.ser.is_open:
            try:
                steer_cmd = f'SERV{servo_angle}\n'
                motor_cmd = f'MOTT{rpm}\n'

                self.ser.write(steer_cmd.encode())
                time.sleep(0.1)  # Delay between commands
                self.ser.write(motor_cmd.encode())

                self.get_logger().info(f"Sent: {steer_cmd.strip()}, {motor_cmd.strip()}")
            except Exception as e:
                self.get_logger().error(f"Serial write failed: {e}")
        else:
            self.get_logger().warn("Serial not connected.")

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
