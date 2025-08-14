#!/usr/bin/env python3
"""
Go2 Robot UI Node

This node provides a comprehensive PyQt interface for Go2 robot control that
communicates with the go2_robot_sdk via ROS topics. It enables:

- Real-time camera video display with QR detection overlay
- Complete robot control interface with predefined action buttons
- Robot status monitoring and performance metrics
- Remote operation and monitoring capabilities

The UI publishes control commands to /webrtc_req topics and subscribes to 
camera streams and robot status from go2_robot_sdk.

Architecture:
- Main UI window with camera display and command controls
- Modular ROS communication layer for robot interaction
- Modular widget design for reusability
- Performance monitoring and diagnostics

This enables intuitive robot control while providing full monitoring capabilities.
"""

import sys
import rclpy
from rclpy.node import Node
from python_qt_binding.QtWidgets import QApplication, QMainWindow, QHBoxLayout, QVBoxLayout, QWidget
from python_qt_binding.QtCore import QTimer
from std_msgs.msg import String

# Import our modular components
from .ros_interface import CameraReceiver, RobotController, StatusMonitor
from .widgets import CameraDisplay, CommandPanel, StatusPanel


class Go2UINode(Node):
    """
    ROS2 node providing comprehensive Go2 robot UI that communicates via ROS transport.
    
    This node creates a complete robot control interface that operates
    with the go2_robot_sdk, enabling flexible deployment scenarios.
    """
    
    def __init__(self):
        super().__init__('go2_robot_ui_node')
        self.get_logger().info('Go2 Robot UI Node starting...')
        self.get_logger().info('Connecting to go2_robot_sdk via ROS topics')
        
        # Create Qt application
        self.app = QApplication(sys.argv)
        
        # Create and show the UI window
        self.ui_window = Go2UIWindow(self)
        self.ui_window.show()
        
        self.get_logger().info('Go2 Robot UI window opened')


class Go2UIWindow(QMainWindow):
    """
    Main UI window for Go2 robot control and monitoring.
    
    Combines modular widgets for video display, robot commands, and status,
    coordinated through ROS communication layer.
    """
    
    def __init__(self, node):
        super().__init__()
        self.node = node
        
        # Initialize ROS communication components
        self.camera_receiver = CameraReceiver(node, '/camera/image_raw')
        self.robot_controller = RobotController(node)
        self.status_monitor = StatusMonitor(node)
        
        # Initialize UI components
        self.camera_display = CameraDisplay()
        self.command_panel = CommandPanel()
        self.status_panel = StatusPanel()
        
        # Set up connection timeout monitoring
        self.connection_timer = QTimer()
        self.connection_timer.timeout.connect(self._check_connections)
        self.connection_timer.start(500)  # Check every 500ms
        
        self.init_ui()
        self.setup_connections()
        
        # Initialize robot state query
        QTimer.singleShot(1000, self._initialize_robot_connection)
    
    def init_ui(self):
        """Initialize the user interface layout and components."""
        self.setWindowTitle('Go2 Robot Control Interface')
        self.setGeometry(100, 100, 1400, 900)
        
        # Create main widget and layout
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QHBoxLayout(main_widget)
        
        # Left side: Camera display (takes most space)
        left_layout = QVBoxLayout()
        left_layout.addWidget(self.camera_display, 4)  # Camera gets most space
        left_layout.addWidget(self.status_panel, 1)    # Status at bottom
        
        # Right side: Command panel
        main_layout.addLayout(left_layout, 3)
        main_layout.addWidget(self.command_panel, 1)
        
        self.node.get_logger().info('UI layout initialized with camera, commands, and status')
    
    def setup_connections(self):
        """Set up signal/slot connections between components."""
        
        # Camera receiver to camera display connections
        self.camera_receiver.frame_ready.connect(self.camera_display.update_frame)
        self.camera_receiver.qr_detected.connect(self.camera_display.highlight_qr)
        self.camera_receiver.qr_detected.connect(self.command_panel.highlight_qr_action)
        # QR detection to robot command execution - automatic action trigger
        self.camera_receiver.qr_detected.connect(
            lambda qr_data, center: self.robot_controller.send_qr_action(qr_data)
        )
        self.camera_receiver.performance_update.connect(self.camera_display.update_performance)
        self.camera_receiver.connection_lost.connect(self.camera_display.on_connection_lost)
        self.camera_receiver.connection_restored.connect(self.camera_display.on_connection_restored)
        
        # Status monitor to status panel connections
        self.status_monitor.robot_state_updated.connect(self.status_panel.update_robot_state)
        self.status_monitor.imu_updated.connect(self.status_panel.update_imu)
        self.status_monitor.joint_states_updated.connect(self.status_panel.update_joints)
        self.status_monitor.connection_status_changed.connect(self.status_panel.update_connection_status)
        
        # Command panel to robot controller connections
        self.command_panel.action_triggered.connect(self.robot_controller.send_robot_action)
        self.command_panel.emergency_stop.connect(self.robot_controller.emergency_stop)
        self.command_panel.qr_action_triggered.connect(self.robot_controller.send_robot_action)
        
        # Robot controller to UI feedback connections
        self.robot_controller.command_sent.connect(self.status_panel.show_command_feedback)
        self.robot_controller.command_failed.connect(self.status_panel.show_error_message)
        
        self.node.get_logger().info('Signal connections established between all components')
    
    def _initialize_robot_connection(self):
        """Initialize connection to robot and query initial state"""
        self.node.get_logger().info('Initializing robot connection and querying state')
        
        # Start monitoring robot status
        self.status_monitor.start_monitoring()
        
        # Query initial robot state
        self.robot_controller.query_robot_state()
    
    def _check_connections(self):
        """Periodically check connection status of all components."""
        self.camera_receiver.check_connection_timeout()
        self.status_monitor.check_connection_timeout()
    
    def closeEvent(self, event):
        """Handle window close event."""
        self.connection_timer.stop()
        self.status_monitor.stop_monitoring()
        self.node.get_logger().info('Go2 Robot UI window closed')
        event.accept()


def main(args=None):
    """
    Main entry point for Go2 robot UI.
    
    Creates a full-featured robot control UI that communicates with go2_robot_sdk
    via ROS topics.
    """
    try:
        # Initialize ROS2
        rclpy.init(args=args)
        
        # Create the robot UI node
        node = Go2UINode()
        
        # Set up ROS spinning timer for Qt integration
        def spin_ros():
            rclpy.spin_once(node, timeout_sec=0.001)
        
        # Qt timer for ROS message handling
        ros_timer = QTimer()
        ros_timer.timeout.connect(spin_ros)
        ros_timer.start(1)  # 1ms interval for responsive ROS handling
        
        try:
            # Run Qt event loop
            exit_code = node.app.exec_()
            node.get_logger().info('Qt application closed')
            
        except KeyboardInterrupt:
            node.get_logger().info('Interrupted by user')
            
    except Exception as e:
        print(f"Error in Go2 robot UI: {e}")
        import traceback
        traceback.print_exc()
        
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()