# Copyright (c) 2024, RoboVerse community
# SPDX-License-Identifier: BSD-3-Clause

import asyncio
import json
import logging
from typing import Callable, Dict, Any

from ...domain.interfaces import IRobotDataReceiver, IRobotController
from ...domain.entities import RobotData, RobotConfig
from .go2_connection import Go2Connection
from ...application.utils.command_generator import gen_command, gen_mov_command
from ...domain.constants import ROBOT_CMD, RTC_TOPIC

logger = logging.getLogger(__name__)


class WebRTCAdapter(IRobotDataReceiver, IRobotController):
    """WebRTC adapter for robot communication"""

    def __init__(self, config: RobotConfig, on_validated_callback: Callable, on_video_frame_callback: Callable = None, event_loop=None):
        self.config = config
        self.connections: Dict[str, Go2Connection] = {}
        self.data_callback: Callable[[RobotData], None] = None
        self.sport_response_callback: Callable[[Dict[str, Any], str], None] = None
        self.webrtc_msgs = asyncio.Queue()
        self.on_validated_callback = on_validated_callback
        self.on_video_frame_callback = on_video_frame_callback
        # Store the event loop (passed from main thread or detect current)
        if event_loop:
            self.main_loop = event_loop
        else:
            try:
                self.main_loop = asyncio.get_running_loop()
            except RuntimeError:
                self.main_loop = None

    async def connect(self, robot_id: str) -> None:
        """Connect to robot via WebRTC"""
        try:
            robot_idx = int(robot_id)
            robot_ip = self.config.robot_ip_list[robot_idx]
            
            conn = Go2Connection(
                robot_ip=robot_ip,
                robot_num=robot_id,
                token=self.config.token,
                on_validated=self._on_validated,
                on_message=self._on_data_channel_message,
                on_video_frame=self.on_video_frame_callback if self.config.enable_video else None,
                decode_lidar=self.config.decode_lidar,
            )
            
            self.connections[robot_id] = conn
            await conn.connect()
            await conn.disableTrafficSaving(True)
            
            logger.info(f"Connected to robot {robot_id} at {robot_ip}")
            
        except Exception as e:
            logger.error(f"Failed to connect to robot {robot_id}: {e}")
            raise

    async def disconnect(self, robot_id: str) -> None:
        """Disconnect from robot"""
        if robot_id in self.connections:
            try:
                # Используем правильный метод для закрытия WebRTC соединения
                connection = self.connections[robot_id]
                if hasattr(connection, 'disconnect'):
                    await connection.disconnect()
                elif hasattr(connection, 'pc') and connection.pc:
                    await connection.pc.close()
                del self.connections[robot_id]
                logger.info(f"Disconnected from robot {robot_id}")
            except Exception as e:
                logger.error(f"Error disconnecting from robot {robot_id}: {e}")

    def set_data_callback(self, callback: Callable[[RobotData], None]) -> None:
        """Set callback for data reception"""
        self.data_callback = callback
    
    def set_sport_response_callback(self, callback: Callable[[Dict[str, Any], str], None]) -> None:
        """Set callback for sport response messages"""
        self.sport_response_callback = callback

    def send_command(self, robot_id: str, command: str) -> None:
        """Send command to robot"""
        logger.info(f"📤 WEBRTC ADAPTER - SENDING COMMAND to robot {robot_id}")
        logger.info(f"   Command: {command[:200]}...")
        
        if robot_id in self.connections:
            try:
                connection = self.connections[robot_id]
                if hasattr(connection, 'data_channel') and connection.data_channel:
                    logger.info(f"   Data Channel State: {connection.data_channel.readyState}")
                    # Use asyncio.run_coroutine_threadsafe to handle cross-thread calls
                    loop = self._get_or_create_event_loop()
                    if loop and loop.is_running():
                        logger.info("   Using async send via event loop")
                        # Schedule the coroutine in the existing loop
                        asyncio.run_coroutine_threadsafe(
                            self._async_send_command(connection, command),
                            loop
                        )
                    else:
                        logger.info("   Using synchronous send")
                        # Fallback to synchronous send
                        connection.data_channel.send(command)
                    logger.info(f"✅ Command successfully sent to robot {robot_id}")
                else:
                    logger.warning(f"❌ No data channel available for robot {robot_id}")
            except Exception as e:
                logger.error(f"❌ Error sending command to robot {robot_id}: {e}")
        else:
            logger.warning(f"❌ Robot {robot_id} not found in connections")

    def _get_or_create_event_loop(self):
        """Get existing event loop or return the main loop"""
        # First try to get the current loop
        try:
            return asyncio.get_running_loop()
        except RuntimeError:
            # If no current loop, return the main loop stored during init
            return self.main_loop

    async def _async_send_command(self, connection, command: str):
        """Async wrapper for sending commands"""
        try:
            if hasattr(connection, 'data_channel') and connection.data_channel:
                logger.info(f"🔄 ASYNC SEND: Sending via data channel...")
                connection.data_channel.send(command)
                logger.info(f"🔄 ASYNC SEND: Message sent successfully")
            else:
                logger.warning(f"🔄 ASYNC SEND: No data channel available")
        except Exception as e:
            logger.error(f"❌ Error in async send command: {e}")

    def send_movement_command(self, robot_id: str, x: float, y: float, z: float) -> None:
        """Send movement command to robot"""
        try:
            command = gen_mov_command(
                round(x, 2), 
                round(y, 2), 
                round(z, 2), 
                self.config.obstacle_avoidance
            )
            self.send_command(robot_id, command)
        except Exception as e:
            logger.error(f"Error sending movement command: {e}")

    def send_stand_up_command(self, robot_id: str) -> None:
        """Send stand up command"""
        try:
            stand_up_cmd = gen_command(ROBOT_CMD["StandUp"])
            self.send_command(robot_id, stand_up_cmd)
            
            move_cmd = gen_command(ROBOT_CMD['BalanceStand'])
            self.send_command(robot_id, move_cmd)
        except Exception as e:
            logger.error(f"Error sending stand up command: {e}")

    def send_stand_down_command(self, robot_id: str) -> None:
        """Send stand down command"""
        try:
            stand_down_cmd = gen_command(ROBOT_CMD["StandDown"])
            self.send_command(robot_id, stand_down_cmd)
        except Exception as e:
            logger.error(f"Error sending stand down command: {e}")

    def send_webrtc_request(self, robot_id: str, api_id: int, parameter: Any, topic: str) -> None:
        """Send WebRTC request"""
        try:
            payload = gen_command(api_id, parameter, topic)
            self.webrtc_msgs.put_nowait(payload)
            logger.debug(f"WebRTC request queued for robot {robot_id}")
        except Exception as e:
            logger.error(f"Error sending WebRTC request: {e}")

    def process_webrtc_commands(self, robot_id: str) -> None:
        """Process WebRTC commands from queue"""
        while True:
            try:
                message = self.webrtc_msgs.get_nowait()
                try:
                    self.send_command(robot_id, message)
                finally:
                    self.webrtc_msgs.task_done()
            except asyncio.QueueEmpty:
                break

    def _on_validated(self, robot_id: str) -> None:
        """Callback after connection validation"""
        try:
            if robot_id in self.connections:
                for topic in RTC_TOPIC.values():
                    self.connections[robot_id].data_channel.send(
                        json.dumps({"type": "subscribe", "topic": topic}))
            
            if self.on_validated_callback:
                self.on_validated_callback(robot_id)
                
        except Exception as e:
            logger.error(f"Error in validated callback: {e}")

    def _on_data_channel_message(self, _, msg: Dict[str, Any], robot_id: str) -> None:
        """Handle incoming data channel messages"""
        try:
            # ENHANCED DEBUG: Log ALL messages with full details
            topic = msg.get('topic', 'NO_TOPIC')
            msg_type = msg.get('type', 'NO_TYPE')
            data = msg.get('data', {})
            
            logger.info(f"🔍 ===== DETAILED WebRTC Message Debug from robot {robot_id} =====")
            logger.info(f"   Topic: '{topic}'")
            logger.info(f"   Type: '{msg_type}'")
            logger.info(f"   All Keys: {list(msg.keys())}")
            
            # Log full message structure for analysis
            try:
                full_msg = json.dumps(msg, indent=2, default=str)
                logger.info(f"   Full Message:\n{full_msg}")
            except Exception as json_error:
                logger.info(f"   Full Message (raw): {str(msg)[:1000]}...")
                logger.info(f"   JSON serialization error: {json_error}")
            
            # Enhanced sport/response detection - check multiple places
            msg_str = str(msg).lower()
            topic_lower = topic.lower()
            
            # Check for sport/response keywords in various parts of the message
            sport_keywords = ['sport', 'response', 'api', '1001', '1002', '1016', '1022', '1008']
            found_keywords = [kw for kw in sport_keywords if kw in msg_str]
            
            if found_keywords:
                logger.info(f"🎯 SPORT-RELATED MESSAGE DETECTED!")
                logger.info(f"   Found keywords: {found_keywords}")
                logger.info(f"   Topic contains sport/response: {'sport' in topic_lower or 'response' in topic_lower}")
                
                # Check data structure for typical response patterns
                if isinstance(data, dict):
                    header = data.get('header', {})
                    identity = header.get('identity', {}) if isinstance(header, dict) else {}
                    status = header.get('status', {}) if isinstance(header, dict) else {}
                    
                    if header:
                        logger.info(f"   Has header structure: YES")
                        logger.info(f"   Identity: {identity}")
                        logger.info(f"   Status: {status}")
                    else:
                        logger.info(f"   Has header structure: NO")
            
            # Expanded list of potential sport response topics
            potential_sport_topics = [
                "rt/api/sport/response",
                "api/sport/response", 
                "sport/response",
                "response",
                "rt/sportresponse",
                "sportresponse",
                "rt/api/response",
                "rt/response", 
                "api/response"
            ]
            
            # Also check if topic contains sport/response patterns
            topic_contains_sport_response = any(pattern in topic_lower for pattern in ['sport', 'response'])
            
            if topic in potential_sport_topics or topic_contains_sport_response:
                logger.info(f"🤖 SPORT RESPONSE TOPIC MATCHED!")
                logger.info(f"   Matched topic: '{topic}'")
                logger.info(f"   Topic contains sport/response: {topic_contains_sport_response}")
                if self.sport_response_callback:
                    logger.info(f"📞 Calling sport response callback...")
                    self.sport_response_callback(msg, robot_id)
                else:
                    logger.warning(f"❌ No sport response callback set!")
            
            # Log why this message was NOT considered a sport response
            logger.info(f"🚫 NOT a sport response - Topic: '{topic}', Keywords found: {found_keywords}")
            
            # Handle other data messages
            if self.data_callback:
                robot_data = RobotData(robot_id=robot_id, timestamp=0.0)
                self.data_callback(msg, robot_id)
                
        except Exception as e:
            logger.error(f"❌ Error processing data channel message: {e}")
            import traceback
            logger.error(f"   Traceback: {traceback.format_exc()}") 