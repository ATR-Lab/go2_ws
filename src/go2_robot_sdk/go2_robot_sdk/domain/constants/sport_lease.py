# Copyright (c) 2024, RoboVerse community
# SPDX-License-Identifier: BSD-3-Clause

"""
Sport lease management constants for GO2 robot control arbitration.
"""

# Sport lease API IDs
SPORT_LEASE_API = {
    "ACQUIRE_LEASE": 3001,
    "RELEASE_LEASE": 3002,
    "GET_LEASE_STATUS": 3003,
}

# Lease status codes
LEASE_STATUS = {
    "AVAILABLE": 0,
    "ACQUIRED": 1,
    "DENIED": 2,
    "EXPIRED": 3,
}

# Default lease timeout (seconds)
DEFAULT_LEASE_TIMEOUT = 300  # 5 minutes
