import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D

class KalmanFilter:
    def __init__(self, q=1e-5, r=1e-4):
        self.x = 0.0  # State estimate
        self.P = 0.01  # Estimate covariance
        self.Q = q    # Process noise
        self.R = r    # Measurement noise

    def update(self, measurement):
        # Prediction update
        self.P += self.Q
        # Kalman gain
        K = self.P / (self.P + self.R)
        # Measurement update
        self.x += K * (measurement - self.x)
        self.P *= (1 - K)
        return self.x

class Pose2DKalmanFilterNode(Node):
    def __init__(self):
        super().__init__('pose2d_kalman_filter_node')

        # Kalman filters for x (lat), y (lon), and theta (heading)
        self.kf_x = KalmanFilter()
        self.kf_y = KalmanFilter()
        self.kf_theta = KalmanFilter()

        print(r"""
 /\_/\  
( o.o ) 
 > ^ <
Kalman Filter Node is ready!
""")

        # Subscriber to raw GPS pose
        self.subscription = self.create_subscription(
            Pose2D,
            'gps/pose2d',
            self.pose_callback,
            10
        )

        # Publisher for filtered pose
        self.publisher = self.create_publisher(
            Pose2D,
            'gps/pose2d_filtered',
            10
        )

    def pose_callback(self, msg):
        filtered_x = self.kf_x.update(msg.x)
        filtered_y = self.kf_y.update(msg.y)
        filtered_theta = self.kf_theta.update(msg.theta)

        filtered_pose = Pose2D()
        filtered_pose.x = filtered_x
        filtered_pose.y = filtered_y
        filtered_pose.theta = filtered_theta

        self.publisher.publish(filtered_pose)

def main(args=None):
    rclpy.init()
    node = Pose2DKalmanFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ =="__main__":
    main()