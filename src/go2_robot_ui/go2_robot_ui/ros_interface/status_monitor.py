"""
Status Monitor Module

Monitors Go2 robot status including robot state, IMU data, joint states, and connection status.
Provides comprehensive robot health and status information for the UI.
"""

import time
from typing import Dict, Any, Optional
from python_qt_binding.QtCore import QObject, pyqtSignal, QTimer
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from go2_interfaces.msg import Go2State, IMU
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry


class StatusMonitor(QObject):
    """
    Monitors robot status and sensor data.
    
    This class subscribes to various robot status topics and provides
    consolidated status information for the UI display.
    
    Signals:
        robot_state_updated: Emitted with robot state data (dict)
        imu_updated: Emitted with IMU data (dict)
        joint_states_updated: Emitted with joint state data (dict)
        connection_status_changed: Emitted when connection status changes (bool)
    """
    
    # Signals for status updates
    robot_state_updated = pyqtSignal(dict)
    imu_updated = pyqtSignal(dict)
    joint_states_updated = pyqtSignal(dict)
    connection_status_changed = pyqtSignal(bool)
    
    def __init__(self, node: Node):
        """
        Initialize status monitor.
        
        Args:
            node: ROS2 node for creating subscriptions
        """
        super().__init__()
        self.node = node
        
        # Connection tracking
        self.last_robot_state_time = 0
        self.last_imu_time = 0
        self.last_joint_state_time = 0
        self.connection_timeout = 5.0  # seconds
        self.is_connected = False
        
        # Status data cache
        self.current_robot_state = {}
        self.current_imu_data = {}
        self.current_joint_data = {}
        
        # Set up QoS profiles
        self.reliable_qos = QoSProfile(depth=10)
        self.best_effort_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Initialize subscriptions (will be created in start_monitoring)
        self.robot_state_sub = None
        self.imu_sub = None
        self.joint_state_sub = None
        self.odometry_sub = None
        
        # Status monitoring timer
        self.status_timer = QTimer()
        self.status_timer.timeout.connect(self._check_connection_status)
        
        node.get_logger().info('Status monitor initialized')
    
    def start_monitoring(self) -> None:
        """Start monitoring robot status topics."""
        try:
            # Robot state subscription
            self.robot_state_sub = self.node.create_subscription(
                Go2State,
                '/go2_states',
                self._robot_state_callback,
                self.reliable_qos
            )
            
            # IMU subscription
            self.imu_sub = self.node.create_subscription(
                IMU,
                '/imu',
                self._imu_callback,
                self.best_effort_qos
            )
            
            # Joint states subscription
            self.joint_state_sub = self.node.create_subscription(
                JointState,
                '/joint_states',
                self._joint_state_callback,
                self.reliable_qos
            )
            
            # Odometry subscription (optional)
            self.odometry_sub = self.node.create_subscription(
                Odometry,
                '/odom',
                self._odometry_callback,
                self.reliable_qos
            )
            
            # Start connection monitoring timer
            self.status_timer.start(1000)  # Check every second
            
            self.node.get_logger().info('Status monitoring started')
            
        except Exception as e:
            self.node.get_logger().error(f'Failed to start status monitoring: {e}')
    
    def stop_monitoring(self) -> None:
        """Stop monitoring robot status topics."""
        try:
            # Stop timer
            self.status_timer.stop()
            
            # Destroy subscriptions
            if self.robot_state_sub:
                self.node.destroy_subscription(self.robot_state_sub)
            if self.imu_sub:
                self.node.destroy_subscription(self.imu_sub)
            if self.joint_state_sub:
                self.node.destroy_subscription(self.joint_state_sub)
            if self.odometry_sub:
                self.node.destroy_subscription(self.odometry_sub)
            
            self.node.get_logger().info('Status monitoring stopped')
            
        except Exception as e:
            self.node.get_logger().error(f'Error stopping status monitoring: {e}')
    
    def _robot_state_callback(self, msg: Go2State) -> None:
        """Process robot state messages."""
        try:
            current_time = time.time()
            self.last_robot_state_time = current_time
            
            # Convert robot state to dictionary
            state_data = {
                'mode': msg.mode,
                'progress': msg.progress,
                'gait_type': msg.gait_type,
                'foot_raise_height': msg.foot_raise_height,
                'position': list(msg.position),
                'body_height': msg.body_height,
                'velocity': list(msg.velocity),
                'range_obstacle': list(msg.range_obstacle),
                'foot_force': list(msg.foot_force),
                'foot_position_body': list(msg.foot_position_body),
                'foot_speed_body': list(msg.foot_speed_body),
                'timestamp': current_time
            }
            
            self.current_robot_state = state_data
            self.robot_state_updated.emit(state_data)
            
            self._update_connection_status(True)
            
        except Exception as e:
            self.node.get_logger().error(f'Error processing robot state: {e}')
    
    def _imu_callback(self, msg: IMU) -> None:
        """Process IMU messages."""
        try:
            current_time = time.time()
            self.last_imu_time = current_time
            
            # Convert IMU data to dictionary
            imu_data = {
                'orientation': {
                    'x': msg.orientation.x,
                    'y': msg.orientation.y,
                    'z': msg.orientation.z,
                    'w': msg.orientation.w
                },
                'angular_velocity': {
                    'x': msg.angular_velocity.x,
                    'y': msg.angular_velocity.y,
                    'z': msg.angular_velocity.z
                },
                'linear_acceleration': {
                    'x': msg.linear_acceleration.x,
                    'y': msg.linear_acceleration.y,
                    'z': msg.linear_acceleration.z
                },
                'temperature': getattr(msg, 'temperature', 0.0),
                'timestamp': current_time
            }
            
            self.current_imu_data = imu_data
            self.imu_updated.emit(imu_data)
            
        except Exception as e:
            self.node.get_logger().error(f'Error processing IMU data: {e}')
    
    def _joint_state_callback(self, msg: JointState) -> None:
        """Process joint state messages."""
        try:
            current_time = time.time()
            self.last_joint_state_time = current_time
            
            # Convert joint states to dictionary
            joint_data = {
                'names': list(msg.name),
                'positions': list(msg.position),
                'velocities': list(msg.velocity) if msg.velocity else [],
                'efforts': list(msg.effort) if msg.effort else [],
                'timestamp': current_time
            }
            
            self.current_joint_data = joint_data
            self.joint_states_updated.emit(joint_data)
            
        except Exception as e:
            self.node.get_logger().error(f'Error processing joint states: {e}')
    
    def _odometry_callback(self, msg: Odometry) -> None:
        """Process odometry messages."""
        try:
            # Add odometry data to robot state if needed
            odom_data = {
                'position': {
                    'x': msg.pose.pose.position.x,
                    'y': msg.pose.pose.position.y,
                    'z': msg.pose.pose.position.z
                },
                'orientation': {
                    'x': msg.pose.pose.orientation.x,
                    'y': msg.pose.pose.orientation.y,
                    'z': msg.pose.pose.orientation.z,
                    'w': msg.pose.pose.orientation.w
                },
                'linear_velocity': {
                    'x': msg.twist.twist.linear.x,
                    'y': msg.twist.twist.linear.y,
                    'z': msg.twist.twist.linear.z
                },
                'angular_velocity': {
                    'x': msg.twist.twist.angular.x,
                    'y': msg.twist.twist.angular.y,
                    'z': msg.twist.twist.angular.z
                }
            }
            
            # Update robot state with odometry
            self.current_robot_state.update({'odometry': odom_data})
            
        except Exception as e:
            self.node.get_logger().error(f'Error processing odometry: {e}')
    
    def _check_connection_status(self) -> None:
        """Check connection status based on message timestamps."""
        current_time = time.time()
        
        # Check if we're receiving data within timeout period
        robot_state_ok = (current_time - self.last_robot_state_time) < self.connection_timeout
        imu_ok = (current_time - self.last_imu_time) < self.connection_timeout
        joint_state_ok = (current_time - self.last_joint_state_time) < self.connection_timeout
        
        # Consider connected if at least robot state is coming through
        new_connection_status = robot_state_ok
        
        if new_connection_status != self.is_connected:
            self._update_connection_status(new_connection_status)
    
    def _update_connection_status(self, connected: bool) -> None:
        """Update connection status and emit signal if changed."""
        if connected != self.is_connected:
            self.is_connected = connected
            self.connection_status_changed.emit(connected)
            
            status_msg = "connected" if connected else "disconnected"
            self.node.get_logger().info(f'Robot status monitoring: {status_msg}')
    
    def check_connection_timeout(self) -> None:
        """External call to check connection timeout."""
        self._check_connection_status()
    
    def get_connection_status(self) -> bool:
        """Get current connection status."""
        return self.is_connected
    
    def get_current_robot_state(self) -> Dict[str, Any]:
        """Get the most recent robot state data."""
        return self.current_robot_state.copy()
    
    def get_current_imu_data(self) -> Dict[str, Any]:
        """Get the most recent IMU data."""
        return self.current_imu_data.copy()
    
    def get_current_joint_data(self) -> Dict[str, Any]:
        """Get the most recent joint state data."""
        return self.current_joint_data.copy()