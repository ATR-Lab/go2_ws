# Copyright (c) 2024, RoboVerse community
# SPDX-License-Identifier: Apache-2.0

"""
Main ROS2 node for dance orchestration.
"""

import logging
import os
import yaml
from typing import Optional, Dict, Any

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from builtin_interfaces.msg import Time
from ament_index_python.packages import get_package_share_directory

from go2_interfaces.srv import ExecuteSingleCommand, StopDanceRoutine
from go2_interfaces.msg import CommandExecutionStatus

from ..application.services.single_command_tracker import SingleCommandTracker
from ..infrastructure.integration.go2_sdk_bridge import Go2SDKBridge
from ..domain.entities.dance_command import CommandExecution

logger = logging.getLogger(__name__)


class DanceOrchestratorNode(Node):
    """Main ROS2 node for dance orchestration"""
    
    def __init__(self):
        super().__init__('dance_orchestrator_node')
        
        # Initialize logging
        logging.basicConfig(level=logging.INFO)
        logger.info("Initializing Dance Orchestrator Node...")
        
        # Load dance commands configuration
        self.command_config = self._load_dance_commands_config()
        logger.info(f"Loaded {len(self.command_config.get('dance_commands', {}))} dance commands from config")
        
        # Integration with go2_robot_sdk
        self.sdk_bridge = Go2SDKBridge(self)
        
        # Single command tracker (Phase 1)
        self.command_tracker = SingleCommandTracker()
        
        # Connect bridge to tracker
        self.sdk_bridge.set_state_callback(
            self.command_tracker.on_robot_state_update
        )
        
        # Set up command sender
        self.command_tracker.set_robot_command_sender(
            self.sdk_bridge.send_dance_command
        )
        
        # ROS2 Services
        self.execute_command_service = self.create_service(
            ExecuteSingleCommand,
            'execute_dance_command',
            self._execute_command_callback
        )
        
        self.stop_command_service = self.create_service(
            StopDanceRoutine,
            'stop_dance_command',
            self._stop_command_callback
        )
        
        # Publishers
        self.status_publisher = self.create_publisher(
            CommandExecutionStatus,
            'dance_command_status',
            10
        )
        
        # Timer for periodic status updates
        self.status_timer = self.create_timer(0.1, self._publish_status_update)
        
        logger.info("Dance Orchestrator Node initialized successfully")
        
    def _load_dance_commands_config(self) -> Dict[str, Any]:
        """Load dance commands configuration from YAML file"""
        try:
            package_share_dir = get_package_share_directory('go2_dance_orchestrator')
            config_path = os.path.join(package_share_dir, 'config', 'dance_commands.yaml')
            
            if not os.path.exists(config_path):
                logger.warning(f"Config file not found at {config_path}, using empty config")
                return {'dance_commands': {}}
            
            with open(config_path, 'r') as file:
                config = yaml.safe_load(file)
                logger.info(f"Successfully loaded config from {config_path}")
                return config
                
        except Exception as e:
            logger.error(f"Error loading config: {e}")
            return {'dance_commands': {}}
    
    def _get_command_duration(self, command_name: str, requested_duration: float = -1.0) -> float:
        """Get command duration from config or use requested override"""
        if requested_duration > 0:
            logger.debug(f"Using override duration {requested_duration}s for {command_name}")
            return requested_duration
            
        # Look up from config
        dance_commands = self.command_config.get('dance_commands', {})
        if command_name not in dance_commands:
            logger.warning(f"Command {command_name} not found in config, using default 5.0s")
            return 5.0  # Default fallback
            
        config_duration = dance_commands[command_name].get('expected_duration', 5.0)
        logger.debug(f"Using config duration {config_duration}s for {command_name}")
        return config_duration
        
    def _execute_command_callback(self, request, response):
        """Handle single command execution requests"""
        logger.info(f"Received command execution request: {request.command_name}")
        
        try:
            # Validate request
            if not request.command_name:
                response.success = False
                response.message = "Command name cannot be empty"
                return response
            
            # Get duration from config or use override
            expected_duration = self._get_command_duration(
                request.command_name, 
                request.expected_duration
            )
            
            # Execute command
            success = self.command_tracker.execute_command(
                command_name=request.command_name,
                expected_duration=expected_duration,
                completion_callback=self._on_command_complete
            )
            
            if success:
                response.success = True
                response.message = f"Command {request.command_name} started successfully (duration: {expected_duration}s)"
                response.estimated_duration = int(expected_duration)
            else:
                response.success = False
                response.message = f"Failed to start command {request.command_name}"
                
        except Exception as e:
            logger.error(f"Error in execute command callback: {e}")
            response.success = False
            response.message = f"Internal error: {str(e)}"
            
        return response
        
    def _stop_command_callback(self, request, response):
        """Handle command stop requests"""
        logger.info("Received command stop request")
        
        try:
            success = self.command_tracker.stop_current_command()
            
            if success:
                response.success = True
                response.message = "Command stopped successfully"
            else:
                response.success = False
                response.message = "No command currently executing"
                
        except Exception as e:
            logger.error(f"Error in stop command callback: {e}")
            response.success = False
            response.message = f"Internal error: {str(e)}"
            
        return response
        
    def _on_command_complete(self, execution: CommandExecution):
        """Handle command completion"""
        logger.info(
            f"Command {execution.command_name} completed - "
            f"Duration: {execution.elapsed_time:.2f}s, "
            f"Reason: {execution.completion_reason}"
        )
        
        # Publish final status update
        self._publish_command_status(execution)
        
    def _publish_status_update(self):
        """Publish periodic status updates"""
        current_execution = self.command_tracker.get_current_execution()
        if current_execution:
            self._publish_command_status(current_execution)
            
    def _publish_command_status(self, execution: CommandExecution):
        """Publish command execution status"""
        status_msg = CommandExecutionStatus()
        status_msg.command_name = execution.command_name
        status_msg.command_id = execution.command_id
        status_msg.status = execution.status.value
        status_msg.progress_percent = execution.progress_percent
        status_msg.elapsed_time = execution.elapsed_time
        status_msg.completion_reason = execution.completion_reason
        
        # Set timestamps
        if execution.start_time:
            start_time = Time()
            start_time.sec = int(execution.start_time)
            start_time.nanosec = int((execution.start_time % 1) * 1e9)
            status_msg.start_time = start_time
            
        if execution.completion_time:
            completion_time = Time()
            completion_time.sec = int(execution.completion_time)
            completion_time.nanosec = int((execution.completion_time % 1) * 1e9)
            status_msg.completion_time = completion_time
        
        self.status_publisher.publish(status_msg)


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        node = DanceOrchestratorNode()
        
        # Use MultiThreadedExecutor for better performance
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        
        logger.info("Dance Orchestrator Node started")
        executor.spin()
        
    except KeyboardInterrupt:
        logger.info("Shutting down Dance Orchestrator Node")
    except Exception as e:
        logger.error(f"Error in Dance Orchestrator Node: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()