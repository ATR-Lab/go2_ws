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
        """Calculate progress percentage using hybrid approach"""
        if self.status == CommandStatus.COMPLETED:
            return 100.0
        elif self.status == CommandStatus.EXECUTING:
            # Method 1: Robot state-based progress (preferred)
            if self.baseline_progress is not None and len(self.progress_history) > 0:
                current_progress = self.progress_history[-1]
                progress_change = abs(current_progress - self.baseline_progress)
                # Robot state indicates progress
                robot_progress = min(95.0, progress_change * 100.0)
                if robot_progress > 10.0:  # Meaningful robot state change
                    return robot_progress
            
            # Method 2: Intelligent timing-based progress (fallback)
            intelligent_durations = {
                "Hello": 3.0, "FingerHeart": 3.0, "Pose": 4.0, "Stretch": 5.0,
                "Dance1": 10.0, "Dance2": 12.0, "WiggleHips": 6.0, "Content": 8.0,
                "FrontFlip": 5.0, "FrontJump": 4.0, "FrontPounce": 4.0, "Handstand": 7.0,
                "Bound": 6.0, "MoonWalk": 8.0, "CrossWalk": 7.0, "OnesidedStep": 5.0,
            }
            expected_duration = intelligent_durations.get(self.command_name, 5.0)
            
            # Smooth progress curve: fast start, slow finish
            time_ratio = min(1.0, self.elapsed_time / expected_duration)
            # Logarithmic progress curve feels more natural
            progress = min(95.0, 100.0 * (1 - (1 - time_ratio) ** 2))
            return max(5.0, progress)  # Always show some progress
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