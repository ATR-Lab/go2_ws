#!/usr/bin/env python3
"""
Test Camera UI Node

This node provides a separated, optimized camera display interface that
communicates with the test_camera_node via ROS topics. It enables:

- Real-time video display with Qt-based rendering
- Performance monitoring and metrics display
- Connection status monitoring and error handling
- Professional UI with modern styling

The UI subscribes to camera streams from /camera/image_raw and provides
an optimized display experience separate from camera processing.

Architecture:
- Qt-based main window with video display
- ROS communication layer for frame reception
- Modular widget design for reusability
- Performance monitoring and diagnostics

This enables headless camera operation while providing full UI capabilities
on local or remote machines.
"""

import sys
import rclpy
from rclpy.node import Node
from python_qt_binding.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget, QHBoxLayout, QLabel
from python_qt_binding.QtCore import QTimer, Qt
from python_qt_binding.QtGui import QFont

# Import our modular components
from .ros_interface import FrameReceiver
from .widgets import CameraDisplay


class TestCameraUINode(Node):
    """
    ROS2 node providing separated camera UI that communicates via ROS transport.
    
    This node creates an optimized camera display interface that operates
    independently from the camera processing node, enabling flexible
    deployment scenarios and improved performance.
    """
    
    def __init__(self):
        super().__init__('test_camera_ui_node')
        self.get_logger().info('Test Camera UI Node starting...')
        self.get_logger().info('Connecting to camera via /camera/image_raw topic')
        
        # Create Qt application
        self.app = QApplication(sys.argv)
        
        # Create and show the UI window
        self.ui_window = TestCameraUIWindow(self)
        self.ui_window.show()
        
        self.get_logger().info('Test Camera UI window opened')


class TestCameraUIWindow(QMainWindow):
    """
    Main UI window for camera display and monitoring.
    
    Provides an optimized Qt-based interface for real-time camera viewing
    with integrated performance metrics and connection monitoring.
    """
    
    def __init__(self, node):
        super().__init__()
        self.node = node
        
        # Initialize ROS communication components
        self.frame_receiver = FrameReceiver(node, '/camera/image_raw')
        
        # Initialize UI components
        self.camera_display = CameraDisplay()
        
        # Set up connection timeout monitoring
        self.connection_timer = QTimer()
        self.connection_timer.timeout.connect(self._check_connection)
        self.connection_timer.start(500)  # Check every 500ms
        
        self.init_ui()
        self.setup_connections()
        
        self.node.get_logger().info('UI initialized and ready for camera frames')
    
    def init_ui(self):
        """Initialize the user interface layout and components."""
        self.setWindowTitle('Test Camera UI - Robot Dog Petting Zoo')
        self.setGeometry(100, 100, 900, 700)
        
        # Create main widget and layout
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QVBoxLayout(main_widget)
        main_layout.setContentsMargins(10, 10, 10, 10)
        main_layout.setSpacing(10)
        
        # Header section
        header_layout = QHBoxLayout()
        
        # Title label
        title_label = QLabel("🤖 Robot Dog Petting Zoo - Camera Monitor")
        title_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        title_font = QFont()
        title_font.setPointSize(16)
        title_font.setBold(True)
        title_label.setFont(title_font)
        title_label.setStyleSheet("""
            QLabel {
                color: #ffffff;
                background-color: #2196f3;
                padding: 12px 20px;
                border-radius: 6px;
                border: 2px solid #1976d2;
            }
        """)
        header_layout.addWidget(title_label, 1)
        
        # Info label
        info_label = QLabel("Optimized Qt-based Camera Display")
        info_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        info_label.setStyleSheet("""
            QLabel {
                color: #666666;
                font-size: 12px;
                padding: 12px 20px;
            }
        """)
        header_layout.addWidget(info_label)
        
        main_layout.addLayout(header_layout)
        
        # Camera display (takes most space)
        main_layout.addWidget(self.camera_display, 1)
        
        # Footer with instructions
        footer_label = QLabel("📋 Instructions: Camera feed will appear automatically when available. Press Alt+F4 or close window to exit.")
        footer_label.setAlignment(Qt.AlignCenter)
        footer_label.setWordWrap(True)
        footer_label.setStyleSheet("""
            QLabel {
                color: #888888;
                background-color: #3a3a3a;
                padding: 8px 15px;
                border-radius: 4px;
                font-size: 11px;
                border: 1px solid #555555;
            }
        """)
        main_layout.addWidget(footer_label)
        
        # Set main window styling
        self.setStyleSheet("""
            QMainWindow {
                background-color: #2b2b2b;
            }
        """)
        
        self.node.get_logger().info('UI layout initialized with professional styling')
    
    def setup_connections(self):
        """Set up signal/slot connections between components."""
        
        # Frame receiver to camera display connections
        self.frame_receiver.frame_ready.connect(self.camera_display.update_frame)
        self.frame_receiver.performance_update.connect(self.camera_display.update_performance)
        self.frame_receiver.connection_lost.connect(self.camera_display.on_connection_lost)
        self.frame_receiver.connection_restored.connect(self.camera_display.on_connection_restored)
        
        self.node.get_logger().info('Signal connections established between all components')
    
    def _check_connection(self):
        """Periodically check frame receiver connection status."""
        self.frame_receiver.check_connection_timeout()
    
    def closeEvent(self, event):
        """Handle window close event."""
        self.connection_timer.stop()
        self.node.get_logger().info('Test Camera UI window closed')
        
        # Shutdown ROS gracefully
        try:
            rclpy.shutdown()
        except:
            pass
        
        event.accept()


def main(args=None):
    """
    Main entry point for separated test camera UI.
    
    Creates an optimized camera display UI that communicates with test_camera_node
    via ROS topics for improved performance and separation of concerns.
    """
    try:
        # Initialize ROS2
        rclpy.init(args=args)
        
        # Create the camera UI node
        node = TestCameraUINode()
        
        # Set up ROS spinning timer for Qt integration
        def spin_ros():
            try:
                rclpy.spin_once(node, timeout_sec=0.001)
            except Exception as e:
                node.get_logger().error(f"ROS spin error: {e}")
        
        # Qt timer for ROS message handling
        ros_timer = QTimer()
        ros_timer.timeout.connect(spin_ros)
        ros_timer.start(1)  # 1ms interval for responsive ROS handling
        
        try:
            # Run Qt event loop
            node.get_logger().info('Starting Qt event loop...')
            exit_code = node.app.exec_()
            node.get_logger().info(f'Qt application closed with exit code: {exit_code}')
            
        except KeyboardInterrupt:
            node.get_logger().info('Interrupted by user')
            
    except Exception as e:
        print(f"Error in test camera UI: {e}")
        import traceback
        traceback.print_exc()
        
    finally:
        try:
            if 'node' in locals():
                node.destroy_node()
            if rclpy.shutdown():
                rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()
