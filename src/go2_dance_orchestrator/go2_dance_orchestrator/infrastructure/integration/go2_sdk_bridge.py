# SPDX-License-Identifier: Apache-2.0

"""
Bridge between dance orchestrator and go2_robot_sdk.
"""

import logging
from typing import Callable, Optional

import rclpy
from rclpy.node import Node
from go2_interfaces.msg import Go2State, WebRtcReq

logger = logging.getLogger(__name__)


class Go2SDKBridge:
    """Bridge between dance orchestrator and go2_robot_sdk"""
    
    def __init__(self, node: Node):
        self.node = node
        
        # Publishers to go2_robot_sdk
        self.webrtc_publisher = node.create_publisher(
            WebRtcReq, 
            '/webrtc_req', 
            10
        )
        
        # Subscribers from go2_robot_sdk  
        self.state_subscriber = node.create_subscription(
            Go2State,
            '/go2_states', 
            self._on_robot_state,
            10
        )
        
        # Callbacks
        self.state_callback: Optional[Callable[[Go2State], None]] = None
        
        logger.info("Go2SDK Bridge initialized")
        
    def send_dance_command(self, command_id: int, parameters: str = "") -> None:
        """Send dance command via WebRTC"""
        msg = WebRtcReq()
        msg.api_id = command_id
        msg.topic = "rt/api/sport/request"  # Sport mode topic for dance commands
        msg.parameter = parameters
        msg.priority = 1  # High priority for dance commands
        
        self.webrtc_publisher.publish(msg)
        logger.debug(f"Published WebRTC command: ID={command_id}, params={parameters}")
        
    def set_state_callback(self, callback: Callable[[Go2State], None]) -> None:
        """Set callback for robot state updates"""
        self.state_callback = callback
        
    def _on_robot_state(self, msg: Go2State) -> None:
        """Forward robot state to registered callback"""
        if self.state_callback:
            try:
                self.state_callback(msg)
            except Exception as e:
                logger.error(f"Error in state callback: {e}")
        
    def is_connected(self) -> bool:
        """Check if connection to go2_robot_sdk is active"""
        # Simple check based on publisher/subscriber count
        return (self.webrtc_publisher.get_subscription_count() > 0 or
                self.state_subscriber.get_publisher_count() > 0)