# Copyright (c) 2024, RoboVerse community
# SPDX-License-Identifier: Apache-2.0

"""
Multi-modal command completion detector.
"""

import numpy as np
from typing import List

from go2_interfaces.msg import Go2State

from ...domain.interfaces.command_tracker import ICompletionDetector
from ...domain.entities.dance_command import CommandExecution


class CommandCompletionDetector(ICompletionDetector):
    """Detects command completion using multiple detection methods"""
    
    def __init__(self):
        # Detection thresholds
        self.progress_threshold = 0.1  # Progress change threshold
        self.stillness_threshold = 0.05  # Velocity threshold for stillness
        self.stillness_duration = 1.0  # Seconds of stillness required
        self.mode_stability_count = 3  # Number of readings for mode stability
        
        # History tracking
        self.velocity_history: List[float] = []
        self.mode_history: List[int] = []
        
    def detect_completion(self, 
                         execution: CommandExecution,
                         current_state: Go2State) -> tuple[bool, str]:
        """
        Detect command completion using multiple methods.
        Returns (is_complete, completion_reason)
        """
        
        # Method 1: Timeout fallback (highest priority)
        if execution.is_timeout_exceeded:
            return True, "timeout_exceeded"
            
        # Method 2: Progress-based detection  
        if self._detect_progress_completion(execution, current_state):
            return True, "progress_returned_to_baseline"
            
        # Method 3: Mode change detection
        if self._detect_mode_completion(execution, current_state):
            return True, "mode_returned_to_baseline"
            
        # Method 4: Movement stillness (for dynamic moves)
        if self._detect_stillness_completion(execution, current_state):
            return True, "movement_stillness_detected"
            
        return False, ""
        
    def _detect_progress_completion(self, execution: CommandExecution, state: Go2State) -> bool:
        """Detect completion based on progress field returning to baseline"""
        if execution.baseline_progress is None:
            return False
            
        # Track progress history
        execution.progress_history.append(state.progress)
        
        # Keep only recent history (last 20 readings)
        if len(execution.progress_history) > 20:
            execution.progress_history.pop(0)
            
        # Check if progress has returned close to baseline
        current_diff = abs(state.progress - execution.baseline_progress)
        if current_diff <= self.progress_threshold:
            # Ensure we've been at baseline for a few readings
            if len(execution.progress_history) >= 5:
                recent_stable = all(
                    abs(p - execution.baseline_progress) <= self.progress_threshold
                    for p in execution.progress_history[-5:]
                )
                return recent_stable
                
        return False
        
    def _detect_mode_completion(self, execution: CommandExecution, state: Go2State) -> bool:
        """Detect completion based on mode returning to baseline"""
        if execution.baseline_mode is None:
            return False
            
        # Track mode history
        self.mode_history.append(state.mode)
        if len(self.mode_history) > 10:
            self.mode_history.pop(0)
            
        # Check if mode has returned to baseline and is stable
        if state.mode == execution.baseline_mode:
            if len(self.mode_history) >= self.mode_stability_count:
                recent_modes = self.mode_history[-self.mode_stability_count:]
                return all(m == execution.baseline_mode for m in recent_modes)
                
        return False
        
    def _detect_stillness_completion(self, execution: CommandExecution, state: Go2State) -> bool:
        """Detect completion based on robot movement stillness"""
        # Calculate overall velocity magnitude
        velocity_magnitude = np.sqrt(sum(v**2 for v in state.velocity))
        
        # Track velocity history
        self.velocity_history.append(velocity_magnitude)
        if len(self.velocity_history) > 30:  # Keep 3 seconds of history at 10Hz
            self.velocity_history.pop(0)
            
        # Check if robot has been still long enough
        if len(self.velocity_history) >= 10:  # At least 1 second of history
            recent_still = all(v <= self.stillness_threshold for v in self.velocity_history[-10:])
            return recent_still
            
        return False
        
    def reset_history(self):
        """Reset detection history for new command"""
        self.velocity_history.clear()
        self.mode_history.clear()