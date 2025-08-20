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
    """Hybrid completion detector: robot state feedback + intelligent timing fallback"""
    
    def __init__(self):
        # Robot state detection thresholds
        self.progress_threshold = 0.1  # Progress change threshold
        self.stillness_threshold = 0.05  # Velocity threshold for stillness
        self.stillness_duration = 1.0  # Seconds of stillness required
        self.mode_stability_count = 3  # Number of readings for mode stability
        
        # Hybrid detection configuration
        self.robot_state_timeout = 3.0  # Wait 3s for robot state before fallback
        self.has_robot_state = False  # Track if robot state data is flowing
        self.last_robot_state_time = None  # Track when we last received state
        
        # Intelligent command duration mapping (based on actual robot behavior)
        self.intelligent_durations = {
            # Basic poses and gestures (quick)
            "Hello": 3.0,
            "FingerHeart": 3.0,
            "Pose": 4.0,
            "Stretch": 5.0,
            
            # Dance moves (longer, expressive)  
            "Dance1": 18.0,
            "Dance2": 12.0,
            "WiggleHips": 6.0,
            "Content": 8.0,
            
            # Athletic moves (medium duration)
            "FrontFlip": 5.0,
            "FrontJump": 4.0,
            "FrontPounce": 4.0,
            "Handstand": 7.0,
            
            # Movement patterns
            "Bound": 6.0,
            "MoonWalk": 8.0,
            "CrossWalk": 7.0,
            "OnesidedStep": 5.0,
            
            # Default for unknown commands
            "_default": 5.0
        }
        
        # History tracking
        self.velocity_history: List[float] = []
        self.mode_history: List[int] = []
        
    def detect_completion(self, 
                         execution: CommandExecution,
                         current_state: Go2State) -> tuple[bool, str]:
        """
        Hybrid completion detection: robot state feedback + intelligent timing fallback.
        Returns (is_complete, completion_reason)
        """
        import time
        
        # Track robot state availability
        if current_state:
            self.has_robot_state = True
            self.last_robot_state_time = time.time()
        
        # Method 1: Robot State Detection (PREFERRED)
        if self.has_robot_state and current_state:
            # Try robot state-based completion detection
            if self._detect_progress_completion(execution, current_state):
                return True, "robot_progress_complete"
                
            if self._detect_mode_completion(execution, current_state):
                return True, "robot_mode_idle"
                
            if self._detect_stillness_completion(execution, current_state):
                return True, "robot_movement_stopped"
        
        # Method 2: Intelligent Timing Fallback
        # Use if: no robot state OR robot state stopped flowing OR reasonable time has passed
        should_use_timing = (
            not self.has_robot_state or  # Never received robot state
            (self.last_robot_state_time and 
             time.time() - self.last_robot_state_time > self.robot_state_timeout) or  # State stopped
            execution.elapsed_time > self.robot_state_timeout  # Give state detection a chance first
        )
        
        if should_use_timing:
            intelligent_duration = self.intelligent_durations.get(
                execution.command_name, 
                self.intelligent_durations["_default"]
            )
            
            if execution.elapsed_time >= intelligent_duration:
                return True, f"intelligent_timing_complete_{intelligent_duration}s"
        
        # Continue waiting
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