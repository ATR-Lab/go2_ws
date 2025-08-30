#!/usr/bin/env python3
"""
Scan Restamper Node
Restamps laser scan messages with current time to fix timing issues in Nav2
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import LaserScan
from builtin_interfaces.msg import Time


class ScanRestamperNode(Node):
    """Node that restamps laser scan messages with current time"""
    
    def __init__(self):
        super().__init__('scan_restamper_node')
        
        # Declare parameters
        self.declare_parameter('input_topic', '/scan')
        self.declare_parameter('output_topic', '/scan_restamped')
        self.declare_parameter('frame_id', '')  # Empty means keep original frame_id
        
        # Get parameters
        self.input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        
        # QoS profiles
        # Use best effort for laser scan data (typical for sensor data)
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Create subscriber and publisher
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            self.input_topic,
            self.scan_callback,
            sensor_qos
        )
        
        self.scan_publisher = self.create_publisher(
            LaserScan,
            self.output_topic,
            sensor_qos
        )
        
        self.get_logger().info(f"Scan restamper started:")
        self.get_logger().info(f"  Input topic: {self.input_topic}")
        self.get_logger().info(f"  Output topic: {self.output_topic}")
        if self.frame_id:
            self.get_logger().info(f"  Frame ID override: {self.frame_id}")
        else:
            self.get_logger().info(f"  Frame ID: keeping original")
    
    def scan_callback(self, msg: LaserScan):
        """Callback to restamp laser scan messages"""
        # Create new message with restamped header
        restamped_msg = LaserScan()
        
        # Copy all data from original message
        restamped_msg.header = msg.header
        restamped_msg.angle_min = msg.angle_min
        restamped_msg.angle_max = msg.angle_max
        restamped_msg.angle_increment = msg.angle_increment
        restamped_msg.time_increment = msg.time_increment
        restamped_msg.scan_time = msg.scan_time
        restamped_msg.range_min = msg.range_min
        restamped_msg.range_max = msg.range_max
        restamped_msg.ranges = msg.ranges
        restamped_msg.intensities = msg.intensities
        
        # Restamp with current time
        current_time = self.get_clock().now()
        restamped_msg.header.stamp = current_time.to_msg()
        
        # Override frame_id if specified
        if self.frame_id:
            restamped_msg.header.frame_id = self.frame_id
        
        # Publish restamped message
        self.scan_publisher.publish(restamped_msg)


def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    
    try:
        node = ScanRestamperNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
