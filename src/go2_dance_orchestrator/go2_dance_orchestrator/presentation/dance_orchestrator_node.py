# SPDX-License-Identifier: Apache-2.0

"""
Main ROS2 node for dance orchestration.
"""

import logging
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from builtin_interfaces.msg import Time

from go2_interfaces.srv import ExecuteSingleCommand, StopDanceRoutine, StartDanceRoutine
from go2_interfaces.msg import CommandExecutionStatus, DanceRoutineStatus

from ..application.services.single_command_tracker import SingleCommandTracker
from ..application.services.dance_sequence_executor import DanceSequenceExecutor, SequenceStatus
from ..infrastructure.integration.go2_sdk_bridge import Go2SDKBridge
from ..domain.entities.dance_command import CommandExecution
from ..domain.enums.command_status import CommandStatus

logger = logging.getLogger(__name__)


class DanceOrchestratorNode(Node):
    """Main ROS2 node for dance orchestration"""
    
    def __init__(self):
        super().__init__('dance_orchestrator_node')
        
        # Initialize logging
        logging.basicConfig(level=logging.INFO)
        logger.info("Initializing Dance Orchestrator Node...")
        
        # Pure detection-based completion - no configuration needed
        
        # Integration with go2_robot_sdk
        self.sdk_bridge = Go2SDKBridge(self)
        
        # Single command tracker (Phase 1)
        self.command_tracker = SingleCommandTracker()
        
        # Dance sequence executor (Phase 2) - reuses the single command tracker
        self.sequence_executor = DanceSequenceExecutor()
        self.sequence_executor.command_tracker = self.command_tracker  # Share the same tracker!
        self.sequence_executor.orchestrator_notifier = self  # Set orchestrator as notifier
        
        # Connect bridge to the shared tracker
        self.sdk_bridge.set_state_callback(
            self.command_tracker.on_robot_state_update
        )
        
        # Set up command sender for the shared tracker
        self.command_tracker.set_robot_command_sender(
            self.sdk_bridge.send_dance_command
        )
        
        # Orchestrator will be notified by sequence executor after advancement
        
        # ROS2 Services
        self.execute_command_service = self.create_service(
            ExecuteSingleCommand,
            'execute_dance_command',
            self._execute_command_callback
        )
        
        self.start_routine_service = self.create_service(
            StartDanceRoutine,
            'start_dance_routine',
            self._start_dance_routine_callback
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
        
        self.routine_status_publisher = self.create_publisher(
            DanceRoutineStatus,
            'dance_routine_status',
            10
        )
        
        # Timer for periodic status updates and completion checking
        self.status_timer = self.create_timer(0.1, self._publish_status_update)
        
        logger.info("Dance Orchestrator Node initialized successfully - Hybrid detection + Sequence execution active")
        
    def _check_command_completion(self, execution: CommandExecution):
        """Check command completion using hybrid detection approach"""
        # Only check the shared tracker since both single commands and sequences use it
        self.command_tracker.check_completion_without_state()
        
    def _execute_command_callback(self, request, response):
        """Handle single command execution requests"""
        logger.info(f"Received command execution request: {request.command_name}")
        
        try:
            # Validate request
            if not request.command_name:
                response.success = False
                response.message = "Command name cannot be empty"
                return response
            
            # Execute command
            success = self.command_tracker.execute_command(
                command_name=request.command_name,
                completion_callback=None  # Orchestrator callback already registered
            )
            
            if success:
                response.success = True
                response.message = f"Command {request.command_name} started successfully (hybrid detection: robot state + intelligent timing)"
            else:
                response.success = False
                response.message = f"Failed to start command {request.command_name}"
                
        except Exception as e:
            logger.error(f"Error in execute command callback: {e}")
            response.success = False
            response.message = f"Internal error: {str(e)}"
            
        return response
            
    def _start_dance_routine_callback(self, request, response):
        """Handle dance routine execution requests"""
        logger.info(f"Received dance routine request: {request.command_sequence}")

        
        try:
            # Validate request
            if not request.command_sequence:
                response.success = False
                response.message = "Command sequence cannot be empty"
                return response
            
            # Check if already executing
            current_sequence = self.sequence_executor.get_current_sequence()
            if current_sequence and current_sequence.status == SequenceStatus.EXECUTING:
                response.success = False
                response.message = f"Sequence '{current_sequence.routine_name}' is still executing"
                return response
            
            # Generate routine name if not provided
            routine_name = request.routine_name
            if not routine_name:
                routine_name = f"routine_{int(time.time())}"
            

            
            # Execute sequence
            success = self.sequence_executor.execute_sequence(
                routine_name=routine_name,
                command_sequence=list(request.command_sequence),
                completion_callback=self._on_sequence_complete
            )
            
            if success:
                response.success = True
                response.message = f"Dance routine '{routine_name}' started successfully with {len(request.command_sequence)} commands"
                response.routine_name = routine_name
            else:
                response.success = False
                response.message = f"Failed to start dance routine"
                
        except Exception as e:
            logger.error(f"Error in start dance routine callback: {e}")
            response.success = False
            response.message = f"Internal error: {str(e)}"
            
        return response
            
    def _stop_command_callback(self, request, response):
        """Handle stop requests (both single commands and sequences)"""
        logger.info("Received stop request")
        
        try:
            # Try to stop sequence first
            sequence_stopped = self.sequence_executor.stop_current_sequence("manual_stop")
            
            # Try to stop single command
            command_stopped = self.command_tracker.stop_current_command("manual_stop")
            
            if sequence_stopped or command_stopped:
                response.success = True
                if sequence_stopped:
                    response.message = "Dance routine stopped successfully"
                else:
                    response.message = "Single command stopped successfully"
            else:
                response.success = False
                response.message = "No command or routine currently executing"
                
        except Exception as e:
            logger.error(f"Error in stop command callback: {e}")
            response.success = False
            response.message = f"Internal error: {str(e)}"
            
        return response
        
    def notify_command_complete(self, execution: CommandExecution):
        """Handle notification of command completion from sequence executor"""
        logger.info(
            f"Command {execution.command_name} completed - "
            f"Duration: {execution.elapsed_time:.2f}s, "
            f"Reason: {execution.completion_reason}"
        )
        
        # Publish final status update
        self._publish_command_status(execution)
        
    def _on_sequence_complete(self, sequence):
        """Handle dance sequence completion"""
        logger.info(
            f"Dance routine '{sequence.routine_name}' completed - "
            f"Duration: {sequence.elapsed_time:.2f}s, "
            f"Commands: {sequence.total_commands}, "
            f"Status: {sequence.status.value}"
        )
        
        # Publish final status update
        self._publish_routine_status(sequence)
        
    def _publish_status_update(self):
        """Publish periodic status updates and check for completion"""
        # Handle single command status
        current_execution = self.command_tracker.get_current_execution()
        if current_execution:
            # Check for completion first
            self._check_command_completion(current_execution)
            # Then publish status
            self._publish_command_status(current_execution)
            
        # Handle sequence status
        current_sequence = self.sequence_executor.get_current_sequence()
        if current_sequence:
            self._publish_routine_status(current_sequence)
            
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
        
    def _publish_routine_status(self, sequence):
        """Publish dance routine execution status"""
        from ..application.services.dance_sequence_executor import SequenceStatus
        
        status_msg = DanceRoutineStatus()
        status_msg.routine_name = sequence.routine_name
        status_msg.command_sequence = sequence.command_sequence
        status_msg.current_command_index = sequence.current_command_index
        status_msg.current_command_name = sequence.current_command_name or ""
        status_msg.total_commands = sequence.total_commands
        status_msg.routine_elapsed_time = sequence.elapsed_time
        status_msg.status = sequence.status.value
        status_msg.completion_reason = sequence.completion_reason
        
        # Get current command progress from single command tracker
        current_execution = self.command_tracker.get_current_execution()
        if (current_execution and 
            current_execution.command_name == sequence.current_command_name):
            status_msg.current_command_progress_percent = current_execution.progress_percent
            # Use current command progress for accurate overall routine progress
            status_msg.routine_progress_percent = sequence.get_routine_progress_percent(current_execution.progress_percent)
        else:
            status_msg.current_command_progress_percent = 0.0
            # Fallback routine progress without current command progress
            status_msg.routine_progress_percent = sequence.routine_progress_percent
        
        self.routine_status_publisher.publish(status_msg)


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