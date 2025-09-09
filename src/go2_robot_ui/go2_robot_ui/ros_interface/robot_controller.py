"""
Robot Controller Module

Handles sending commands to the Go2 robot via WebRTC requests.
Provides high-level interface for robot actions and emergency controls.
"""

import time
from typing import Dict, Any, Optional
from python_qt_binding.QtCore import QObject, pyqtSignal
from rclpy.node import Node
from rclpy.qos import QoSProfile
from go2_interfaces.msg import WebRtcReq
from std_msgs.msg import String
from go2_robot_sdk.domain.constants.robot_commands import ROBOT_CMD


class RobotController(QObject):
    """
    Handles robot command transmission and control.
    
    This class provides a high-level interface for sending commands to the Go2 robot
    via WebRTC requests, including predefined actions and emergency controls.
    
    Signals:
        command_sent: Emitted when command is successfully sent (str: action_name)
        command_failed: Emitted when command fails (str: error_message)
    """
    
    # Signals for command feedback
    command_sent = pyqtSignal(str)
    command_failed = pyqtSignal(str)
    
    # Robot command constants (imported from go2_robot_sdk)
    ROBOT_COMMANDS = ROBOT_CMD
    
    # QR code to action mapping
    QR_ACTION_MAP = {
        "HELLO": "Hello",
        "HI": "Hello", 
        "WAVE": "Hello",
        "SIT": "Sit",
        "SITDOWN": "Sit",
        "STAND": "StandUp",
        "STANDUP": "StandUp",
        "UP": "StandUp",
        "DANCE": "Dance1",
        "DANCE1": "Dance1",
        "DANCE2": "Dance2",
        "PARTY": "Dance1",
        "STRETCH": "Stretch",
        "EXERCISE": "Stretch",
        "FLIP": "FrontFlip",
        "FRONTFLIP": "FrontFlip",
        "JUMP": "FrontJump",
        "HEART": "FingerHeart",
        "LOVE": "FingerHeart",
        "HIPS": "WiggleHips",
        "WIGGLE": "WiggleHips",
        "HANDSTAND": "Handstand",
        "MOONWALK": "MoonWalk",
        "MOON": "MoonWalk",
        "STOP": "StopMove",
        "EMERGENCY": "StopMove",
        "BALANCE": "BalanceStand",
    }
    
    def __init__(self, node: Node):
        """
        Initialize robot controller.
        
        Args:
            node: ROS2 node for creating publishers
        """
        super().__init__()
        self.node = node
        
        # Set up QoS profile
        self.qos_profile = QoSProfile(depth=10)
        
        # Create WebRTC request publisher
        self.webrtc_pub = node.create_publisher(
            WebRtcReq,
            '/webrtc_req',
            self.qos_profile
        )
        
        # Command state tracking
        self.last_command_time = 0
        self.command_cooldown = 0.5  # Minimum time between commands (seconds)
        self.emergency_cooldown = 0.1  # Shorter cooldown for emergency commands
        
        node.get_logger().info('Robot controller initialized')
        node.get_logger().info(f'Available actions: {list(self.ROBOT_COMMANDS.keys())}')
    
    def send_robot_action(self, action_name: str, is_emergency: bool = False) -> bool:
        """
        Send a robot action command.
        
        Args:
            action_name: Name of the action to execute
            is_emergency: Whether this is an emergency command (bypasses cooldown)
            
        Returns:
            True if command was sent successfully, False otherwise
        """
        try:
            # Check if action exists
            if action_name not in self.ROBOT_COMMANDS:
                error_msg = f"Unknown action: {action_name}"
                self.node.get_logger().error(error_msg)
                self.command_failed.emit(error_msg)
                return False
            
            # Check command cooldown (unless emergency)
            current_time = time.time()
            cooldown = self.emergency_cooldown if is_emergency else self.command_cooldown
            
            if current_time - self.last_command_time < cooldown:
                error_msg = f"Command cooldown active, please wait"
                self.node.get_logger().warn(error_msg)
                self.command_failed.emit(error_msg)
                return False
            
            # Create and send WebRTC request
            api_id = self.ROBOT_COMMANDS[action_name]
            success = self._send_webrtc_request(api_id, action_name)
            
            if success:
                self.last_command_time = current_time
                self.command_sent.emit(action_name)
                self.node.get_logger().info(f'Command sent: {action_name} (API ID: {api_id})')
                return True
            else:
                error_msg = f"Failed to send command: {action_name}"
                self.command_failed.emit(error_msg)
                return False
                
        except Exception as e:
            error_msg = f"Error sending robot action: {e}"
            self.node.get_logger().error(error_msg)
            self.command_failed.emit(error_msg)
            return False
    
    def send_qr_action(self, qr_data: str) -> bool:
        """
        Send robot action based on QR code data.
        
        Args:
            qr_data: QR code content
            
        Returns:
            True if action was sent successfully, False otherwise
        """
        try:
            # Convert QR data to uppercase for matching
            qr_upper = qr_data.upper().strip()
            
            # Check if QR code maps to an action
            if qr_upper in self.QR_ACTION_MAP:
                action_name = self.QR_ACTION_MAP[qr_upper]
                self.node.get_logger().info(f'QR code "{qr_data}" mapped to action: {action_name}')
                return self.send_robot_action(action_name)
            else:
                # Try direct action name match
                if qr_upper in [name.upper() for name in self.ROBOT_COMMANDS.keys()]:
                    # Find the exact case match
                    action_name = next(name for name in self.ROBOT_COMMANDS.keys() 
                                     if name.upper() == qr_upper)
                    self.node.get_logger().info(f'QR code "{qr_data}" matched direct action: {action_name}')
                    return self.send_robot_action(action_name)
                else:
                    error_msg = f"QR code '{qr_data}' does not match any known action"
                    self.node.get_logger().warn(error_msg)
                    self.command_failed.emit(error_msg)
                    return False
                    
        except Exception as e:
            error_msg = f"Error processing QR action: {e}"
            self.node.get_logger().error(error_msg)
            self.command_failed.emit(error_msg)
            return False
    
    def emergency_stop(self) -> bool:
        """
        Send emergency stop command to robot.
        
        Returns:
            True if command was sent successfully, False otherwise
        """
        self.node.get_logger().warn('EMERGENCY STOP requested')
        return self.send_robot_action("StopMove", is_emergency=True)
    
    def query_robot_state(self) -> bool:
        """
        Query the current robot state.
        
        Returns:
            True if query was sent successfully, False otherwise
        """
        try:
            # Send GetState command
            success = self._send_webrtc_request(1034, "GetState")
            if success:
                self.node.get_logger().info('Robot state query sent')
            return success
        except Exception as e:
            error_msg = f"Error querying robot state: {e}"
            self.node.get_logger().error(error_msg)
            self.command_failed.emit(error_msg)
            return False
    
    def _send_webrtc_request(self, api_id: int, action_name: str, parameter: str = "") -> bool:
        """
        Send WebRTC request to robot.
        
        Args:
            api_id: API command ID
            action_name: Human-readable action name
            parameter: Optional command parameter
            
        Returns:
            True if request was sent successfully, False otherwise
        """
        try:
            # Create WebRTC request message
            req = WebRtcReq()
            req.id = 0  # Auto-assigned
            req.api_id = api_id
            req.topic = "rt/api/sport/request"  # Standard sport mode topic
            req.parameter = parameter if parameter else str(api_id)
            req.priority = 1 if action_name == "StopMove" else 0  # High priority for stop
            
            # Publish the request
            self.webrtc_pub.publish(req)
            
            self.node.get_logger().debug(f'WebRTC request sent - API ID: {api_id}, Topic: {req.topic}')
            return True
            
        except Exception as e:
            self.node.get_logger().error(f'Failed to send WebRTC request: {e}')
            return False
    
    def get_available_actions(self) -> Dict[str, int]:
        """Get dictionary of available robot actions and their API IDs."""
        return self.ROBOT_COMMANDS.copy()
    
    def get_qr_action_map(self) -> Dict[str, str]:
        """Get dictionary mapping QR codes to robot actions."""
        return self.QR_ACTION_MAP.copy()
    
    def is_valid_action(self, action_name: str) -> bool:
        """Check if an action name is valid."""
        return action_name in self.ROBOT_COMMANDS