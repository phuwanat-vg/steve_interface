#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float32, Int32 

import math
import numpy as np

# --- Constants ---
EARTH_RADIUS = 6371000.0


WAYPOINTS = [
    (14.871901, 102.020667),  # Example: New York City
     # Example: Los Angeles
   
]

class WaypointFollowerNode(Node):


    def __init__(self):
     
        super().__init__('waypoint_follower_node')
        self.get_logger().info('Waypoint Follower Node Initializing...')


        self.pose2d_subscriber = self.create_subscription(
            Pose2D,
            '/gps/pose2d_filtered',  
            self.pose2d_callback,10
        )

        self.speed_publisher = self.create_publisher(
            Int32,
            'speed', # Topic for rear wheel speed in RPM
            10
        )
        self.get_logger().info('Publishing to /speed (Float64)')

        # Publish steering angle (radians)
        self.steer_publisher = self.create_publisher(
            Float32,
            'steer', # Topic for steering angle in radians
            10
        )
        self.get_logger().info('Publishing to /steer (Float64)')

        # --- State Variables ---
        self.current_latitude = None
        self.current_longitude = None

        # Robot's current heading in radians (0 to 2*PI, North is 0/2pi, East pi/2, etc.)
        # Assumes theta from Pose2D is already in radians and relative to True North.
        self.current_heading_rad = None

        self.waypoints = WAYPOINTS
        self.current_waypoint_index = 0

        # --- Navigation Parameters (set directly) ---
        # Tolerance in meters for considering a waypoint reached.
        self.waypoint_tolerance_m = 2.0
        self.get_logger().info(f'Waypoint tolerance set to: {self.waypoint_tolerance_m} m')

        # Desired constant linear speed for the robot in m/s (used in Pure Pursuit).
        self.desired_linear_speed = 0.3
        self.get_logger().info(f'Desired linear speed set to: {self.desired_linear_speed} m/s')

        # Lookahead distance for Pure Pursuit in meters.
        # This is Ld in the Pure Pursuit formula. Adjust based on robot speed and desired path smoothness.
        self.lookahead_distance_m = 2.0
        self.get_logger().info(f'Lookahead distance set to: {self.lookahead_distance_m} m')

        # --- Car-like Robot Specific Parameters ---
        self.wheel_radius = 0.125 * 0.5  
        self.get_logger().info(f'Wheel radius set to: {self.wheel_radius} m')

        # Distance between the front and rear axles (wheelbase) in meters.
        self.wheel_base = 0.225 # meters (e.g., 50 cm wheelbase) - IMPORTANT: Set your robot's actual wheelbase
        self.get_logger().info(f'Wheelbase set to: {self.wheel_base} m')

        # Maximum steering angle in radians (e.g., 30 degrees converted to radians).
        # This limits the physical turning capability of the front wheels.
        self.max_steer_angle_rad = math.radians(40.0)
        self.get_logger().info(f'Max steering angle set to: {math.degrees(self.max_steer_angle_rad):.2f} deg')

        # --- Control Loop Timer ---
        # Periodically call the follow_waypoints method to update commands
        self.timer_period = 0.1 # seconds (10 Hz)
        self.timer = self.create_timer(self.timer_period, self.follow_waypoints)
        self.get_logger().info(f'Control loop timer set to {self.timer_period} seconds.')

    def pose2d_callback(self, msg: Pose2D):
        
        self.current_latitude = msg.x
        self.current_longitude = msg.y
        # Assuming theta from Pose2D is already in radians and relative to True North
        # (0 = North, increasing counter-clockwise, i.e., positive Z-axis rotation).
        self.current_heading_rad = msg.theta

        # Normalize heading to be within [0, 2*PI) just in case it comes in other ranges
        self.current_heading_rad = self.current_heading_rad % (2 * math.pi)
        if self.current_heading_rad < 0:
            self.current_heading_rad += 2 * math.pi

        # self.get_logger().debug(f'Pose2D: Lat={self.current_latitude:.6f}, Lon={self.current_longitude:.6f}, Heading={math.degrees(self.current_heading_rad):.2f} deg')

    def haversine_distance(self, lat1, lon1, lat2, lon2):
      
        lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])

        dlat = lat2 - lat1
        dlon = lon2 - lon1
        a = math.sin(dlat / 2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        distance = EARTH_RADIUS * c
        return distance

    def bearing_to_waypoint(self, lat1, lon1, lat2, lon2):
        """
        Calculates the true bearing (azimuth) from point 1 to point 2.
        Args:
            lat1, lon1: Latitude and longitude of point 1 in degrees.
            lat2, lon2: Latitude and longitude of point 2 in degrees.
        Returns:
            Bearing in radians (0 to 2*PI, North is 0, East is PI/2).
        """
        lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])

        dlon = lon2 - lon1
        x = math.sin(dlon) * math.cos(lat2)
        y = math.cos(lat1) * math.sin(lat2) - (math.sin(lat1) * math.cos(lat2) * math.cos(dlon))
        bearing_rad = math.atan2(x, y)

        # Normalize bearing to be within [0, 2*PI)
        if bearing_rad < 0:
            bearing_rad += 2 * math.pi
        return bearing_rad

    def follow_waypoints(self):
        """
        Main control loop for waypoint following using Pure Pursuit.
        This function is called periodically by the timer.
        """
        # Ensure we have current Pose2D data
        if self.current_latitude is None or self.current_longitude is None or self.current_heading_rad is None:
            self.get_logger().warn('Waiting for Pose2D data (latitude, longitude, or heading)...')
            rpm_msg = Int32()
            steer_msg = Float32()
            rpm_msg.data = 0
            steer_msg.data = 0.0
            self.speed_publisher.publish(rpm_msg) # Stop robot
            self.steer_publisher.publish(steer_msg) # Center steering
            return

        # Check if all waypoints have been reached
        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info('All waypoints reached! Stopping robot.')
            rpm_msg = Int32()
            steer_msg = Float32()
            rpm_msg.data = 0
            steer_msg.data = 0.0
            self.speed_publisher.publish(rpm_msg) # Stop robot
            self.steer_publisher.publish(steer_msg) # Center steering
            return

        target_waypoint = self.waypoints[self.current_waypoint_index]
        target_lat, target_lon = target_waypoint

        # Calculate distance to current target waypoint
        distance_to_waypoint = self.haversine_distance(
            self.current_latitude, self.current_longitude,
            target_lat, target_lon
        )
        # self.get_logger().debug(f'Distance to waypoint {self.current_waypoint_index}: {distance_to_waypoint:.2f} m')

        # Check if the current waypoint is reached (within tolerance)
        if distance_to_waypoint < self.waypoint_tolerance_m:
            self.get_logger().info(f'Waypoint {self.current_waypoint_index} reached!')
            self.current_waypoint_index += 1 # Move to the next waypoint
            if self.current_waypoint_index >= len(self.waypoints):
                self.get_logger().info('All waypoints finished. Stopping.')
                rpm_msg = Int32()
                steer_msg = Float32()
                rpm_msg.data = 0
                steer_msg.data = 0.0
                self.speed_publisher.publish(rpm_msg) # Stop robot
                self.steer_publisher.publish(steer_msg) # Center steeringself.speed_publisher.publish(Int32(data=0.0)) # Stop robot
                return
            else:
                self.get_logger().info(f'Proceeding to waypoint {self.current_waypoint_index}...')
                # No explicit stop needed here, Pure Pursuit will guide towards the next point.

        # --- Pure Pursuit Control Logic ---
        current_speed_rpm = 0.0
        current_steer_angle_rad = 0.0

        # Calculate desired true bearing to the current target waypoint
        desired_bearing_rad = self.bearing_to_waypoint(
            self.current_latitude, self.current_longitude,
            target_lat, target_lon
        )
        # self.get_logger().debug(f'Desired Bearing: {math.degrees(desired_bearing_rad):.2f} deg')
        # self.get_logger().debug(f'Current Heading: {math.degrees(self.current_heading_rad):.2f} deg')

        # Calculate the angle (alpha) between the robot's forward vector and the
        # vector to the lookahead point (current target waypoint).
        # This is essentially the heading error.
        alpha = desired_bearing_rad - self.current_heading_rad

        # Normalize alpha to be within (-PI, PI] for correct shortest path rotation
        if alpha > math.pi:
            alpha -= 2 * math.pi
        elif alpha < -math.pi:
            alpha += 2 * math.pi

        # self.get_logger().debug(f'Alpha (Heading Error): {math.degrees(alpha):.2f} deg')

        # Calculate steering angle using the Pure Pursuit formula for car-like robots:
        # delta = atan2(2 * L * sin(alpha), Ld)
        # Where L = wheel_base, Ld = lookahead_distance_m
        if self.lookahead_distance_m > 0 and self.wheel_base > 0:
            current_steer_angle_rad = math.atan2(
                2 * self.wheel_base * math.sin(alpha),
                self.lookahead_distance_m
            )
        else:
            self.get_logger().warn('Lookahead distance or Wheelbase is zero/negative. Setting steering angle to 0.')
            current_steer_angle_rad = 0.0

        # Clamp steering angle to prevent excessively sharp turns beyond physical limits
        current_steer_angle_rad = np.clip(
            current_steer_angle_rad,
            -self.max_steer_angle_rad,
            self.max_steer_angle_rad
        )

        # Calculate rear wheel RPM from desired linear speed
        if self.wheel_radius > 0:
            # Linear speed (m/s) to RPM conversion:
            # RPM = (Linear Speed / (2 * pi * Wheel Radius)) * 60 seconds/minute
            current_speed_rpm = (self.desired_linear_speed / (2 * math.pi * self.wheel_radius)) * 60
        else:
            self.get_logger().warn('Wheel radius is zero or negative. Setting speed RPM to 0.')
            current_speed_rpm = 0.0

        # Publish the calculated speed and steering angle
        rpm_msg = Int32()
        steer_msg = Float32()
        rpm_msg.data = int(current_speed_rpm)
        steer_msg.data = float(current_steer_angle_rad/3.14159*180)
            
        self.speed_publisher.publish(rpm_msg)
        self.steer_publisher.publish(steer_msg)

        self.get_logger().info(
            f'WP {self.current_waypoint_index}: Dist={distance_to_waypoint:.2f}m, '
            f'Alpha={math.degrees(alpha):.2f}deg, '
            f'Speed_RPM={current_speed_rpm:.2f}, Steer_Rad={current_steer_angle_rad:.2f} ({math.degrees(current_steer_angle_rad):.2f} deg)'
        )


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped cleanly.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
