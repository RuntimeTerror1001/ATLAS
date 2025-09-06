#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class ImuCovarianceFixerNode(Node):
    def __init__(self):
        super().__init__('imu_covariance_fixer')

        # Create publisher
        self.publisher = self.create_publisher(Imu, '/atlas/imu', 10)

        # Create subscriber
        self.subscriber = self.create_subscription(Imu, '/atlas/imu/data', self.listener_cb, 10)

        self.get_logger().info('IMU Covariance Fixer Node started. Relaying fixed topic')

    def listener_cb(self, msg):
        new_msg = msg

        # Orientation Covariance (roll, pitch, yaw)
        new_msg.orientation_covariance[0] = 0.01  # Variance for roll
        new_msg.orientation_covariance[4] = 0.01  # Variance for pitch
        new_msg.orientation_covariance[8] = 0.01  # Variance for yaw

        # Angular Velocity Covariance (v_roll, v_pitch, v_yaw)
        new_msg.angular_velocity_covariance[0] = 0.001 # Variance for v_roll
        new_msg.angular_velocity_covariance[4] = 0.001 # Variance for v_pitch
        new_msg.angular_velocity_covariance[8] = 0.001 # Variance for v_yaw

        # Linear Acceleration Covariance (ax, ay, az)
        new_msg.linear_acceleration_covariance[0] = 0.1   # Variance for ax
        new_msg.linear_acceleration_covariance[4] = 0.1   # Variance for ay
        new_msg.linear_acceleration_covariance[8] = 0.1   # Variance for az

        self.publisher.publish(new_msg)

def main(argc=None):
    rclpy.init(args=argc)
    node = ImuCovarianceFixerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()