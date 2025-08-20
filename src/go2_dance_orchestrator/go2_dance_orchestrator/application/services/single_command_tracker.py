# Copyright (c) 2024, RoboVerse community
# SPDX-License-Identifier: Apache-2.0

"""
Single command tracker implementation.
"""

import logging
from typing import Callable, Optional

from go2_interfaces.msg import Go2State

from ...domain.interfaces.command_tracker import ICommandTracker
from ...domain.entities.dance_command import CommandExecution
from ...domain.enums.command_status import CommandStatus
from .completion_detector import CommandCompletionDetector

# Import robot commands - we'll need to get these from go2_robot_sdk
# For now, we'll define the ones we need
ROBOT_CMD = {
    "Hello": 1016,
    "FrontFlip": 1030,
    "Dance1": 1022,
    "Dance2": 1023,
    "Handstand": 1301,
    "WiggleHips": 1033,
    "FingerHeart": 1036,
    "MoonWalk": 1305,
    "Stretch": 1017,
    "Pose": 1028,
}

logger = logging.getLogger(__name__)


class SingleCommandTracker(ICommandTracker):
    """Tracks execution of individual dance commands"""
    
    def __init__(self):
        self.completion_detector = CommandCompletionDetector()
        self.current_execution: Optional[CommandExecution] = None
        self.completion_callback: Optional[Callable[[CommandExecution], None]] = None
        self.robot_command_sender: Optional[Callable[[int, str], None]] = None
        
    def set_robot_command_sender(self, sender: Callable[[int, str], None]):
        """Set callback for sending commands to robot"""
        self.robot_command_sender = sender
        
    def execute_command(self, 
                       command_name: str,
                       completion_callback: Callable[[CommandExecution], None]) -> bool:
        """Execute a single dance command with tracking"""
        
        # Check if command is valid
        command_id = ROBOT_CMD.get(command_name)
        if not command_id:
            logger.error(f"Unknown command: {command_name}")
            return False
            
        # Check if another command is already executing
        if (self.current_execution and 
            self.current_execution.status == CommandStatus.EXECUTING):
            logger.warning(f"Command {self.current_execution.command_name} is still executing")
            return False
            
        # Create execution tracker
        self.current_execution = CommandExecution(
            command_name=command_name,
            command_id=command_id
        )
        
        self.completion_callback = completion_callback
        
        # Reset completion detector history
        self.completion_detector.reset_history()
        
        # Send command to robot
        if self.robot_command_sender:
            try:
                self.robot_command_sender(command_id, "")
                logger.info(f"Sent command {command_name} (ID: {command_id}) to robot")
            except Exception as e:
                logger.error(f"Failed to send command to robot: {e}")
                self.current_execution.fail(f"send_error: {e}")
                return False
        else:
            logger.error("No robot command sender configured")
            return False
        
        # Start tracking
        self.current_execution.start()
        logger.info(f"Started tracking command {command_name} (using robot state feedback)")
        
        return True
        
    def on_robot_state_update(self, state: Go2State) -> None:
        """Process robot state updates for completion detection"""
        if (not self.current_execution or 
            self.current_execution.status != CommandStatus.EXECUTING):
            return
            
        # Store baseline on first state update after command start
        if self.current_execution.baseline_progress is None:
            self.current_execution.baseline_progress = state.progress
            self.current_execution.baseline_mode = state.mode
            logger.info(f"BASELINE SET - Progress: {state.progress}, Mode: {state.mode}")
            return
            
        # Log current state for debugging
        logger.info(f"ROBOT STATE - Progress: {state.progress} (baseline: {self.current_execution.baseline_progress}), "
                   f"Mode: {state.mode} (baseline: {self.current_execution.baseline_mode}), "
                   f"Velocity: {state.velocity}")
        
        # Check for completion
        is_complete, reason = self.completion_detector.detect_completion(
            self.current_execution, state
        )
        
        if is_complete:
            logger.info(f"COMPLETION DETECTED - Reason: {reason}")
            self._complete_command(reason)
        else:
            # Log why not complete every few seconds
            if int(self.current_execution.elapsed_time) % 3 == 0:
                logger.info(f"Still executing - elapsed: {self.current_execution.elapsed_time:.1f}s")
            
    def get_current_execution(self) -> Optional[CommandExecution]:
        """Get currently executing command"""
        return self.current_execution
        
    def stop_current_command(self, reason: str = "manual_stop") -> bool:
        """Stop currently executing command"""
        if (self.current_execution and 
            self.current_execution.status == CommandStatus.EXECUTING):
            self._complete_command(reason)
            return True
        return False
            
    def _complete_command(self, reason: str):
        """Mark command as completed and trigger callback"""
        if not self.current_execution:
            return
            
        self.current_execution.complete(reason)
        
        logger.info(
            f"Command {self.current_execution.command_name} completed in "
            f"{self.current_execution.elapsed_time:.2f}s (reason: {reason})"
        )
        
        if self.completion_callback:
            try:
                self.completion_callback(self.current_execution)
            except Exception as e:
                logger.error(f"Error in completion callback: {e}")
                
        # Reset for next command
        self.current_execution = None
        self.completion_callback = None