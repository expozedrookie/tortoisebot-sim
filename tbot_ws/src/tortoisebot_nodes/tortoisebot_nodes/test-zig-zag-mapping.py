import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
import math

class DistanceEstimator(Node):
    def __init__(self):
        super().__init__('distance_estimator')

        # Publisher to publish the estimated distance
        self.distance_pub = self.create_publisher(Float32, '/estimated_distance', 10)

        # Subscriber to Odometry data
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',  # Make sure to change this if your odom topic is different
            self.odom_callback,
            10
        )

        # Initialize variables to track the distance
        self.last_position = None
        self.total_distance = 0.0

        self.get_logger().info("Distance Estimation Node started")

    def odom_callback(self, msg):
        # Get the current position from the odometry message
        current_position = msg.pose.pose.position

        # If this is the first message, initialize the last_position
        if self.last_position is None:
            self.last_position = current_position
            return

        # Calculate the distance traveled since the last position
        delta_x = current_position.x - self.last_position.x
        delta_y = current_position.y - self.last_position.y
        distance_travelled = math.sqrt(delta_x**2 + delta_y**2)

        # Add the traveled distance to the total distance
        self.total_distance += distance_travelled

        # Publish the total distance
        distance_msg = Float32()
        distance_msg.data = self.total_distance
        self.distance_pub.publish(distance_msg)

        # Update the last_position to the current position
        self.last_position = current_position

        # Log the current total distance
        self.get_logger().info(f"Total distance: {self.total_distance:.2f} meters")

def main(args=None):
    rclpy.init(args=args)
    node = DistanceEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup and shutdown
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
