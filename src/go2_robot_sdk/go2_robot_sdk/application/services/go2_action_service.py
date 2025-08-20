# Copyright (c) 2024, RoboVerse community
# SPDX-License-Identifier: BSD-3-Clause

import asyncio
import json
import logging
import time
import uuid
from typing import Dict, Optional, Callable

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from go2_interfaces.srv import ExecuteAction
from go2_interfaces.msg import Req, Res, SportModeState
from ...domain.constants import RTC_TOPIC, ROBOT_CMD

logger = logging.getLogger(__name__)


class Go2ActionService(Node):
    """ROS2 Service for executing GO2 robot actions with completion feedback"""

    def __init__(self):
        super().__init__('go2_action_service')
        
        # QoS profile
        self.qos_profile = QoSProfile(depth=10)
        
        # Service server
        self.action_service = self.create_service(
            ExecuteAction, 
            '/go2/execute_action', 
            self.execute_action_callback
        )
        
        # Publishers
        self.sport_request_pub = self.create_publisher(
            Req, 
            RTC_TOPIC["SPORT_MOD"], 
            self.qos_profile
        )
        
        # Subscribers
        self.sport_response_sub = self.create_subscription(
            Res,
            RTC_TOPIC["SPORT_MOD_RESPONSE"],
            self.sport_response_callback,
            self.qos_profile
        )
        
        self.sport_state_sub = self.create_subscription(
            SportModeState,
            RTC_TOPIC["LF_SPORT_MOD_STATE"],
            self.sport_state_callback,
            self.qos_profile
        )
        
        # Pending requests tracking
        self.pending_requests: Dict[str, Dict] = {}
        
        # Current robot state
        self.current_sport_state: Optional[SportModeState] = None
        
        self.get_logger().info("GO2 Action Service initialized")

    async def execute_action_callback(self, request, response):
        """Handle action execution requests"""
        start_time = time.time()
        request_uuid = str(uuid.uuid4())
        
        try:
            # Validate action ID
            if not self._is_valid_action_id(request.action_id):
                response.success = False
                response.error_code = 4
                response.error_message = f"Invalid action ID: {request.action_id}"
                response.execution_time = time.time() - start_time
                return response
            
            # Set default timeout
            timeout = request.timeout if request.timeout > 0 else 30.0
            
            # Create future for async response handling
            future = asyncio.Future()
            
            # Store pending request
            self.pending_requests[request_uuid] = {
                'future': future,
                'start_time': start_time,
                'action_id': request.action_id,
                'timeout': timeout,
                'parameters': request.parameters
            }
            
            # Send sport request
            sport_req = Req()
            sport_req.uuid = request_uuid
            
            # Build request body
            request_body = {
                "api_id": request.action_id,
                "parameter": json.loads(request.parameters) if request.parameters else {}
            }
            sport_req.body = json.dumps(request_body)
            
            self.sport_request_pub.publish(sport_req)
            self.get_logger().info(f"Sent action request {request.action_id} with UUID {request_uuid}")
            
            # Wait for response with timeout
            try:
                result = await asyncio.wait_for(future, timeout=timeout)
                
                response.success = result['success']
                response.error_code = result['error_code']
                response.error_message = result['error_message']
                response.execution_time = time.time() - start_time
                
            except asyncio.TimeoutError:
                response.success = False
                response.error_code = 1
                response.error_message = f"Action timed out after {timeout} seconds"
                response.execution_time = time.time() - start_time
                
                # Clean up pending request
                if request_uuid in self.pending_requests:
                    del self.pending_requests[request_uuid]
            
        except Exception as e:
            self.get_logger().error(f"Error executing action: {e}")
            response.success = False
            response.error_code = 5
            response.error_message = f"Communication error: {str(e)}"
            response.execution_time = time.time() - start_time
        
        return response

    def sport_response_callback(self, msg: Res):
        """Handle sport mode responses"""
        try:
            request_uuid = msg.uuid
            
            if request_uuid not in self.pending_requests:
                self.get_logger().debug(f"Received response for unknown UUID: {request_uuid}")
                return
            
            pending_req = self.pending_requests[request_uuid]
            
            # Parse response body
            try:
                response_data = json.loads(msg.body) if msg.body else {}
            except json.JSONDecodeError:
                response_data = {}
            
            # Determine success based on response
            error_code = response_data.get('error_code', 0)
            success = error_code == 0
            
            result = {
                'success': success,
                'error_code': error_code,
                'error_message': response_data.get('error_message', 'Action completed' if success else 'Action failed')
            }
            
            # Set future result
            if not pending_req['future'].done():
                pending_req['future'].set_result(result)
            
            # Clean up
            del self.pending_requests[request_uuid]
            
            self.get_logger().info(f"Action {pending_req['action_id']} completed with result: {result}")
            
        except Exception as e:
            self.get_logger().error(f"Error processing sport response: {e}")

    def sport_state_callback(self, msg: SportModeState):
        """Handle sport mode state updates for fallback monitoring"""
        self.current_sport_state = msg
        
        # Check for error codes in state
        if msg.error_code != 0:
            self.get_logger().warning(f"Sport mode error detected: {msg.error_code}")
            
            # Check if any pending requests might be affected
            current_time = time.time()
            for request_uuid, pending_req in list(self.pending_requests.items()):
                # If request is recent and we have an error, it might be related
                if current_time - pending_req['start_time'] < 5.0:
                    if not pending_req['future'].done():
                        result = {
                            'success': False,
                            'error_code': 2,
                            'error_message': f"Robot reported error: {msg.error_code}"
                        }
                        pending_req['future'].set_result(result)
                        del self.pending_requests[request_uuid]
                        break

    def _is_valid_action_id(self, action_id: int) -> bool:
        """Validate if action ID is known"""
        return action_id in ROBOT_CMD.values()

    def _cleanup_expired_requests(self):
        """Clean up expired requests"""
        current_time = time.time()
        expired_uuids = []
        
        for request_uuid, pending_req in self.pending_requests.items():
            if current_time - pending_req['start_time'] > pending_req['timeout']:
                expired_uuids.append(request_uuid)
                
                if not pending_req['future'].done():
                    result = {
                        'success': False,
                        'error_code': 1,
                        'error_message': 'Request expired'
                    }
                    pending_req['future'].set_result(result)
        
        for uuid_to_remove in expired_uuids:
            del self.pending_requests[uuid_to_remove]


def main():
    """Main entry point"""
    rclpy.init()
    
    try:
        service_node = Go2ActionService()
        
        # Create executor for async handling
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(service_node)
        
        try:
            executor.spin()
        except KeyboardInterrupt:
            pass
        finally:
            executor.shutdown()
            service_node.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
