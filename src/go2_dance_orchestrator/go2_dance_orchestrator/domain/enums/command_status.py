# Copyright (c) 2024, RoboVerse community
# SPDX-License-Identifier: Apache-2.0

"""
Command status enums for dance orchestration.
"""

from enum import IntEnum


class CommandStatus(IntEnum):
    """Status of command execution"""
    PENDING = 0
    EXECUTING = 1
    COMPLETED = 2
    FAILED = 3
    TIMEOUT = 4


class CompletionReason(IntEnum):
    """Reason for command completion"""
    PROGRESS_DETECTION = 0
    MODE_CHANGE = 1
    MOVEMENT_STILLNESS = 2
    TIMEOUT_EXCEEDED = 3
    ERROR = 4
    MANUAL_STOP = 5