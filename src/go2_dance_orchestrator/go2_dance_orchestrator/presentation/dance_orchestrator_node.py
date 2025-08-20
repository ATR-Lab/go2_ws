# Copyright (c) 2024, RoboVerse community
# SPDX-License-Identifier: Apache-2.0

"""
Main ROS2 node for dance orchestration.
"""

import logging
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from builtin_interfaces.msg import Time

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
        
    def _execute_command_callback(self, request, response):
        """Handle single command execution requests"""
        logger.info(f"Received command execution request: {request.command_name}")
        
        try:
            # Validate request
            if not request.command_name:
                response.success = False
                response.message = "Command name cannot be empty"
                return response
                
            if request.expected_duration <= 0:
                response.success = False
                response.message = "Expected duration must be positive"
                return response
            
            # Execute command
            success = self.command_tracker.execute_command(
                command_name=request.command_name,
                expected_duration=request.expected_duration,
                completion_callback=self._on_command_complete
            )
            
            if success:
                response.success = True
                response.message = f"Command {request.command_name} started successfully"
                response.estimated_duration = int(request.expected_duration)
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