# SPDX-License-Identifier: Apache-2.0

"""
Sport Response Service - handles sport command responses from the robot
"""

import json
import logging
from typing import Dict, Any

logger = logging.getLogger(__name__)


class SportResponseService:
    """Service for processing sport command responses"""
    
    def __init__(self):
        self.pending_commands: Dict[str, Dict] = {}  # Track commands by ID
        
    def handle_sport_response(self, msg: Dict[str, Any], robot_id: str) -> None:
        """Process incoming sport response messages"""
        try:
            logger.info(f"🤖 SPORT RESPONSE from robot {robot_id}: {json.dumps(msg, indent=2)}")
            
            # Extract response data
            data = msg.get('data', {})
            if isinstance(data, str):
                try:
                    data = json.loads(data)
                except json.JSONDecodeError:
                    logger.warn(f"Could not parse response data as JSON: {data}")
                    return
            
            # Check if this matches the expected Unitree API response structure
            header = data.get('header', {})
            identity = header.get('identity', {})
            status = header.get('status', {})
            
            api_id = identity.get('api_id', 'unknown')
            request_id = identity.get('id', 'unknown')
            status_code = status.get('code', -1)
            status_message = status.get('message', '')
            
            logger.info(f"📨 Sport Response Details:")
            logger.info(f"   API ID: {api_id}")
            logger.info(f"   Request ID: {request_id}")  
            logger.info(f"   Status Code: {status_code} {'✅ SUCCESS' if status_code == 0 else '❌ FAILED'}")
            if status_message:
                logger.info(f"   Status Message: {status_message}")
            
            # TODO: Implement command correlation and callback mechanism
            # For now, just log that we successfully received the response
            
        except Exception as e:
            logger.error(f"Error processing sport response: {e}")
    
    def register_pending_command(self, command_id: str, api_id: int, callback=None):
        """Register a pending command for response correlation"""
        self.pending_commands[command_id] = {
            'api_id': api_id,
            'callback': callback
        }
        logger.debug(f"Registered pending command: {command_id} (API {api_id})")
        
    def get_pending_command(self, command_id: str) -> Dict:
        """Get and remove a pending command"""
        return self.pending_commands.pop(command_id, None)