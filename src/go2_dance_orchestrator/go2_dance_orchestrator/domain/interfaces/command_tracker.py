# Copyright (c) 2024, RoboVerse community
# SPDX-License-Identifier: Apache-2.0

"""
Interface definitions for command tracking.
"""

from abc import ABC, abstractmethod
from typing import Callable, Optional

from go2_interfaces.msg import Go2State

from ..entities.dance_command import CommandExecution


class ICommandTracker(ABC):
    """Interface for tracking dance command execution"""
    
    @abstractmethod
    def execute_command(self, 
                       command_name: str,
                       completion_callback: Callable[[CommandExecution], None]) -> bool:
        """Execute a single dance command with tracking"""
        pass
    
    @abstractmethod
    def on_robot_state_update(self, state: Go2State) -> None:
        """Process robot state updates for completion detection"""
        pass
    
    @abstractmethod
    def get_current_execution(self) -> Optional[CommandExecution]:
        """Get currently executing command"""
        pass
    
    @abstractmethod
    def stop_current_command(self) -> bool:
        """Stop currently executing command"""
        pass


class ICompletionDetector(ABC):
    """Interface for detecting command completion"""
    
    @abstractmethod
    def detect_completion(self, 
                         execution: CommandExecution,
                         current_state: Go2State) -> tuple[bool, str]:
        """
        Detect if command is complete.
        Returns (is_complete, completion_reason)
        """
        pass