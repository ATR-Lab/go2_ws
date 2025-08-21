# Copyright (c) 2024, RoboVerse community
# SPDX-License-Identifier: Apache-2.0

"""
Dance sequence executor for managing multi-command routines.
"""

import logging
import time
import threading
from typing import List, Callable, Optional
from enum import Enum

from ...domain.entities.dance_command import CommandExecution
from ...domain.enums.command_status import CommandStatus
from .single_command_tracker import SingleCommandTracker

logger = logging.getLogger(__name__)


class SequenceStatus(Enum):
    """Enumeration for sequence execution status"""
    PENDING = "PENDING"
    EXECUTING = "EXECUTING" 
    COMPLETED = "COMPLETED"
    FAILED = "FAILED"
    CANCELLED = "CANCELLED"


class DanceSequenceExecution:
    """Tracks execution of a dance sequence"""
    
    def __init__(self, routine_name: str, command_sequence: List[str]):
        self.routine_name = routine_name
        self.command_sequence = command_sequence
        self.current_command_index = -1  # -1 = not started
        self.status = SequenceStatus.PENDING
        self.start_time = None
        self.completion_time = None
        self.completion_reason = ""
        self.failed_command = None
        
    @property
    def current_command_name(self) -> Optional[str]:
        """Get name of currently executing command"""
        if 0 <= self.current_command_index < len(self.command_sequence):
            return self.command_sequence[self.current_command_index]
        return None
    
    @property
    def total_commands(self) -> int:
        """Total number of commands in sequence"""
        return len(self.command_sequence)
        
    @property
    def elapsed_time(self) -> float:
        """Get elapsed time since sequence start"""
        if not self.start_time:
            return 0.0
        end_time = self.completion_time or time.time()
        return end_time - self.start_time
    
    def get_routine_progress_percent(self, current_command_progress: float = 0.0) -> float:
        """Calculate overall routine progress percentage with current command progress"""
        if self.status == SequenceStatus.COMPLETED:
            return 100.0
        elif self.status == SequenceStatus.EXECUTING:
            if self.current_command_index < 0:
                return 0.0
            # Progress = (completed_commands / total_commands) * 100
            completed_commands = max(0, self.current_command_index)
            base_progress = (completed_commands / self.total_commands) * 100.0
            
            # Add current command progress (scaled to its portion)
            if completed_commands < self.total_commands:
                current_portion = 100.0 / self.total_commands  # Each command is 1/N of total
                base_progress += (current_command_progress / 100.0) * current_portion
                
            return min(95.0, base_progress)  # Cap at 95% until truly complete
        else:
            return 0.0
            
    @property
    def routine_progress_percent(self) -> float:
        """Calculate overall routine progress percentage (fallback without current command progress)"""
        return self.get_routine_progress_percent(0.0)
            
    def start(self):
        """Start sequence execution"""
        self.start_time = time.time()
        self.status = SequenceStatus.EXECUTING
        self.current_command_index = 0
        
    def complete(self, reason: str):
        """Mark sequence as completed"""
        self.completion_time = time.time()
        self.status = SequenceStatus.COMPLETED
        self.completion_reason = reason
        self.current_command_index = len(self.command_sequence)  # Beyond last command
        
    def fail(self, reason: str, failed_command: str):
        """Mark sequence as failed"""
        self.completion_time = time.time()
        self.status = SequenceStatus.FAILED
        self.completion_reason = reason
        self.failed_command = failed_command
        
    def cancel(self, reason: str = "user_cancelled"):
        """Cancel sequence execution"""
        self.completion_time = time.time()
        self.status = SequenceStatus.CANCELLED
        self.completion_reason = reason
        
    def advance_to_next_command(self):
        """Move to next command in sequence"""
        self.current_command_index += 1


class DanceSequenceExecutor:
    """Executes dance sequences using a shared single command tracker"""
    
    def __init__(self):
        self.command_tracker = None  # Will be injected from orchestrator
        self.current_sequence: Optional[DanceSequenceExecution] = None
        self.sequence_completion_callback: Optional[Callable[[DanceSequenceExecution], None]] = None
        
        # Command transition timing (inspired by UI cooldown system)
        self.base_transition_delay = 0.8  # Base 800ms delay between commands
        self.command_specific_delays = {
            # Athletic moves need more recovery time
            "FrontFlip": 1.2,
            "Handstand": 1.5, 
            "FrontJump": 1.0,
            "FrontPounce": 1.0,
            
            # Dance moves can be quicker
            "Dance1": 1.0,
            "Dance2": 1.0,
            "WiggleHips": 0.6,
            "Content": 0.8,
            
            # Basic moves are fastest
            "Hello": 0.5,
            "FingerHeart": 0.5,
            "Stretch": 0.6,
            "Pose": 0.4,
        }
        
    def _get_transition_delay(self, completed_command: str) -> float:
        """Get the appropriate delay after a command completes before starting next command"""
        return self.command_specific_delays.get(completed_command, self.base_transition_delay)
        
    def execute_sequence(self, 
                        routine_name: str,
                        command_sequence: List[str],
                        completion_callback: Callable[[DanceSequenceExecution], None]) -> bool:
        """Execute a dance sequence"""
        
        # Validate that command tracker is set
        if not self.command_tracker:
            logger.error("Command tracker not set - cannot execute sequence")
            return False
        
        # Validate sequence
        if not command_sequence:
            logger.error("Empty command sequence provided")
            return False
            
        # Check if already executing
        if (self.current_sequence and 
            self.current_sequence.status == SequenceStatus.EXECUTING):
            logger.warning(f"Sequence {self.current_sequence.routine_name} is still executing")
            return False
            
        # Create sequence execution
        self.current_sequence = DanceSequenceExecution(routine_name, command_sequence)
        self.sequence_completion_callback = completion_callback
        
        logger.info(f"Starting dance sequence '{routine_name}': {command_sequence}")
        self.current_sequence.start()
        
        # Start first command
        self._execute_next_command()
        
        return True
        
    def _execute_next_command(self):
        """Execute the next command in the current sequence"""
        if not self.current_sequence or self.current_sequence.status != SequenceStatus.EXECUTING:
            return
            
        # Check if sequence is complete
        if self.current_sequence.current_command_index >= self.current_sequence.total_commands:
            self._complete_sequence("all_commands_completed")
            return
            
        # Execute current command
        command_name = self.current_sequence.current_command_name
        logger.info(f"Sequence '{self.current_sequence.routine_name}': Starting command {self.current_sequence.current_command_index + 1}/{self.current_sequence.total_commands}: '{command_name}'")
        
        success = self.command_tracker.execute_command(
            command_name=command_name,
            completion_callback=self._on_command_complete
        )
        
        if not success:
            self._fail_sequence(f"failed_to_start_command_{command_name}", command_name)
            
    def _on_command_complete(self, execution: CommandExecution):
        """Handle completion of a single command within the sequence"""
        if not self.current_sequence:
            return
            
        logger.info(f"Sequence '{self.current_sequence.routine_name}': Command '{execution.command_name}' completed in {execution.elapsed_time:.1f}s (reason: {execution.completion_reason})")
        
        # Check if command failed
        if execution.status == CommandStatus.FAILED:
            self._fail_sequence(f"command_failed_{execution.completion_reason}", execution.command_name)
            return
            
        # Move to next command
        self.current_sequence.advance_to_next_command()
        
        # Apply transition delay before next command (like UI cooldown system)
        transition_delay = self._get_transition_delay(execution.command_name)
        logger.info(f"Sequence '{self.current_sequence.routine_name}': Waiting {transition_delay}s before next command (robot recovery time)")
        
        # Schedule next command after delay using timer thread
        timer = threading.Timer(transition_delay, self._execute_next_command)
        timer.daemon = True  # Don't prevent program exit
        timer.start()
        
    def _complete_sequence(self, reason: str):
        """Complete the current sequence"""
        if not self.current_sequence:
            return
            
        self.current_sequence.complete(reason)
        logger.info(f"Dance sequence '{self.current_sequence.routine_name}' completed in {self.current_sequence.elapsed_time:.1f}s")
        
        if self.sequence_completion_callback:
            try:
                self.sequence_completion_callback(self.current_sequence)
            except Exception as e:
                logger.error(f"Error in sequence completion callback: {e}")
                
    def _fail_sequence(self, reason: str, failed_command: str):
        """Fail the current sequence"""
        if not self.current_sequence:
            return
            
        self.current_sequence.fail(reason, failed_command)
        logger.error(f"Dance sequence '{self.current_sequence.routine_name}' failed at command '{failed_command}': {reason}")
        
        if self.sequence_completion_callback:
            try:
                self.sequence_completion_callback(self.current_sequence)
            except Exception as e:
                logger.error(f"Error in sequence completion callback: {e}")
                
    def stop_current_sequence(self, reason: str = "manual_stop") -> bool:
        """Stop currently executing sequence"""
        if not self.current_sequence or self.current_sequence.status != SequenceStatus.EXECUTING:
            return False
            
        # Stop current command first
        self.command_tracker.stop_current_command(reason)
        
        # Cancel sequence
        self.current_sequence.cancel(reason)
        logger.info(f"Dance sequence '{self.current_sequence.routine_name}' cancelled: {reason}")
        
        if self.sequence_completion_callback:
            try:
                self.sequence_completion_callback(self.current_sequence)
            except Exception as e:
                logger.error(f"Error in sequence completion callback: {e}")
                
        return True
        
    def get_current_sequence(self) -> Optional[DanceSequenceExecution]:
        """Get currently executing sequence"""
        return self.current_sequence