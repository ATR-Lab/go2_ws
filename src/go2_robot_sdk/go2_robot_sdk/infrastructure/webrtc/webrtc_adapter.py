# Copyright (c) 2024, RoboVerse community
# SPDX-License-Identifier: BSD-3-Clause

import asyncio
import json
import logging
from typing import Callable, Dict, Any
from enum import Enum

from ...domain.interfaces import IRobotDataReceiver, IRobotController
from ...domain.entities import RobotData, RobotConfig
from .go2_connection import Go2Connection
from ...application.utils.command_generator import gen_command, gen_mov_command
from ...domain.constants import ROBOT_CMD, RTC_TOPIC

class CommandPriority(Enum):
    """Phase 2 Optimization: Command priority classification for direct bypass system"""
    CRITICAL = "critical"    # Direct send - movement, emergency, safety
    HIGH = "high"           # Fast processing - state changes, mode switches  
    NORMAL = "normal"       # Standard queue - configuration, entertainment

logger = logging.getLogger(__name__)


class WebRTCAdapter(IRobotDataReceiver, IRobotController):
    """WebRTC adapter for robot communication"""

    def __init__(self, config: RobotConfig, on_validated_callback: Callable, on_video_frame_callback: Callable = None, event_loop=None):
        self.config = config
        self.connections: Dict[str, Go2Connection] = {}
        self.data_callback: Callable[[RobotData], None] = None
        self.sport_response_callback: Callable[[Dict[str, Any], str], None] = None
        self.webrtc_msgs = asyncio.Queue()
        # Phase 2 Optimization: Command priority classification for bypass system
        self._critical_commands = {
            1008,  # Move - direct movement commands
            1003,  # StopMove - emergency stops 
            1001,  # Damp - safety/damping
            1006,  # RecoveryStand - emergency recovery
        }
        self._high_priority_commands = {
            1004, 1005,  # StandUp, StandDown - basic state changes
            1002,  # BalanceStand - stance control
            1011,  # SwitchGait - movement mode changes
            1027,  # SwitchJoystick - control mode changes
        }
        
        # Phase 2 Optimization: Async message processing queue
        # Separates WebRTC callback thread from heavy message processing
        self._message_processing_queue = asyncio.Queue(maxsize=1000)
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
        # Phase 1 Optimization: Reduced logging verbosity in critical command path
        # Only log at debug level to minimize I/O overhead during high-frequency commands
        logger.debug(f"Sending command to robot {robot_id}: {command[:50]}...")
        
        if robot_id in self.connections:
            try:
                connection = self.connections[robot_id]
                if hasattr(connection, 'data_channel') and connection.data_channel:
                    # Use asyncio.run_coroutine_threadsafe to handle cross-thread calls
                    loop = self._get_or_create_event_loop()
                    if loop and loop.is_running():
                        # Schedule the coroutine in the existing loop
                        asyncio.run_coroutine_threadsafe(
                            self._async_send_command(connection, command),
                            loop
                        )
                    else:
                        # Fallback to synchronous send
                        connection.data_channel.send(command)
                else:
                    logger.warning(f"No data channel available for robot {robot_id}")
            except Exception as e:
                logger.error(f"Error sending command to robot {robot_id}: {e}")
        else:
            logger.warning(f"Robot {robot_id} not found in connections")

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
                # Phase 1 Optimization: Removed verbose logging from async send path
                # Critical performance path should minimize I/O operations
                connection.data_channel.send(command)
            else:
                logger.warning(f"No data channel available in async send")
        except Exception as e:
            logger.error(f"Error in async send command: {e}")

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

    def _classify_command_priority(self, api_id: int) -> CommandPriority:
        """Phase 2 Optimization: Classify command priority for bypass system"""
        if api_id in self._critical_commands:
            return CommandPriority.CRITICAL
        elif api_id in self._high_priority_commands:
            return CommandPriority.HIGH
        else:
            return CommandPriority.NORMAL

    def send_webrtc_request(self, robot_id: str, api_id: int, parameter: Any, topic: str) -> None:
        """Send WebRTC request with Phase 2 priority-based routing"""
        try:
            payload = gen_command(api_id, parameter, topic)
            priority = self._classify_command_priority(api_id)
            
            # Phase 2 Optimization: Direct bypass for critical commands
            # Critical commands (movement, emergency) skip queue for minimum latency
            if priority == CommandPriority.CRITICAL:
                logger.debug(f"Critical command {api_id} bypassing queue for robot {robot_id}")
                self.send_command(robot_id, payload)
            else:
                # High and normal priority commands use queue for batched processing
                self.webrtc_msgs.put_nowait(payload)
                logger.debug(f"{priority.value.title()} priority command {api_id} queued for robot {robot_id}")
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
        """Handle incoming data channel messages with Phase 2 async processing"""
        try:
            # Phase 2 Optimization: Fast-path message classification and async queuing
            # Minimize work in WebRTC callback thread - just classify and queue
            topic = msg.get('topic', 'NO_TOPIC')
            
            # Fast classification without heavy processing
            topic_lower = topic.lower()
            is_sport_response = any(pattern in topic_lower for pattern in ['sport', 'response'])
            
            # Queue message for async processing instead of blocking WebRTC thread
            message_data = {
                'msg': msg,
                'robot_id': robot_id,
                'topic': topic,
                'is_sport_response': is_sport_response
            }
            
            # Non-blocking queue put with fallback if queue is full
            try:
                self._message_processing_queue.put_nowait(message_data)
                logger.debug(f"Message queued for async processing: {topic}")
            except asyncio.QueueFull:
                # Drop oldest message to prevent blocking - prioritize real-time performance
                try:
                    self._message_processing_queue.get_nowait()
                    self._message_processing_queue.put_nowait(message_data)
                    logger.warning(f"Message queue full - dropped oldest message for {robot_id}")
                except asyncio.QueueEmpty:
                    pass  # Queue became empty, our message should fit now
                
        except Exception as e:
            logger.error(f"Error in fast-path message processing for robot {robot_id}: {e}")

    async def _process_message_queue_worker(self) -> None:
        """Phase 2 Optimization: Async worker for processing queued messages"""
        while True:
            try:
                # Wait for message from queue
                message_data = await self._message_processing_queue.get()
                
                # Process the message with full logic (moved from callback)
                await self._process_queued_message(message_data)
                
                # Mark task as done
                self._message_processing_queue.task_done()
                
            except Exception as e:
                logger.error(f"Error in message queue worker: {e}")
                # Continue processing other messages
                
    async def _process_queued_message(self, message_data: Dict[str, Any]) -> None:
        """Phase 2 Optimization: Process individual queued message asynchronously"""
        try:
            msg = message_data['msg']
            robot_id = message_data['robot_id']
            topic = message_data['topic']
            is_sport_response = message_data['is_sport_response']
            
            # Handle sport response messages
            if is_sport_response:
                potential_sport_topics = [
                    "rt/api/sport/response", "api/sport/response", "sport/response",
                    "response", "rt/sportresponse", "sportresponse",
                    "rt/api/response", "rt/response", "api/response"
                ]
                
                if topic.lower() in [t.lower() for t in potential_sport_topics]:
                    if self.sport_response_callback:
                        self.sport_response_callback(msg, robot_id)
                    else:
                        logger.warning(f"No sport response callback set for robot {robot_id}")
            
            # Handle other data messages
            if self.data_callback:
                self.data_callback(msg, robot_id)
                
        except Exception as e:
            logger.error(f"Error processing queued message for robot {robot_id}: {e}") 