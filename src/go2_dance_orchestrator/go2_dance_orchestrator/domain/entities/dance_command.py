# Copyright (c) 2024, RoboVerse community
# SPDX-License-Identifier: Apache-2.0

"""
Dance command entities for orchestration system.
"""

import time
from dataclasses import dataclass, field
from typing import List, Optional

from ..enums.command_status import CommandStatus


@dataclass
class CommandExecution:
    """Tracks execution of a single dance command"""
    command_name: str
    command_id: int
    start_time: Optional[float] = None
    completion_time: Optional[float] = None
    status: CommandStatus = CommandStatus.PENDING
    
    # State tracking for completion detection
    baseline_progress: Optional[float] = None
    baseline_mode: Optional[int] = None
    progress_history: List[float] = field(default_factory=list)
    completion_reason: str = ""
    
    # Configuration parameters
    delay_before: float = 0.0
    parameters: str = ""
    priority: int = 0
    
    @property
    def elapsed_time(self) -> float:
        """Calculate elapsed time since command started"""
        if not self.start_time:
            return 0.0
        end_time = self.completion_time or time.time()
        return end_time - self.start_time
    
    @property
    def progress_percent(self) -> float:
        """Calculate progress percentage based on robot state feedback"""
        if self.status == CommandStatus.COMPLETED:
            return 100.0
        elif self.status == CommandStatus.EXECUTING:
            # Real progress based on robot state changes
            if self.baseline_progress is not None and len(self.progress_history) > 0:
                current_progress = self.progress_history[-1]
                progress_change = abs(current_progress - self.baseline_progress)
                # Heuristic: more change from baseline = more progress
                return min(95.0, progress_change * 100.0)
            else:
                # Just started - robot state baseline not yet established
                return 5.0  
        else:
            return 0.0
            
    def start(self):
        """Start command execution"""
        self.start_time = time.time()
        self.status = CommandStatus.EXECUTING
        
    def complete(self, reason: str):
        """Mark command as completed"""
        self.completion_time = time.time()
        self.status = CommandStatus.COMPLETED
        self.completion_reason = reason
        
    def fail(self, reason: str):
        """Mark command as failed"""
        self.completion_time = time.time()
        self.status = CommandStatus.FAILED
        self.completion_reason = reason