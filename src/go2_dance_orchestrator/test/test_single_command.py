#!/usr/bin/env python3
# Copyright (c) 2024, RoboVerse community
# SPDX-License-Identifier: Apache-2.0

"""
Test script for single dance command execution.
"""

import time
import rclpy
from rclpy.node import Node
from go2_interfaces.srv import ExecuteSingleCommand, StopDanceRoutine
from go2_interfaces.msg import CommandExecutionStatus


class DanceCommandTester(Node):
    """Test node for dance command execution"""
    
    def __init__(self):
        super().__init__('dance_command_tester')
        
        # Service clients
        self.execute_client = self.create_client(
            ExecuteSingleCommand, 
            'execute_dance_command'
        )
        self.stop_client = self.create_client(
            StopDanceRoutine,
            'stop_dance_command'
        )
        
        # Status subscriber
        self.status_subscriber = self.create_subscription(
            CommandExecutionStatus,
            'dance_command_status',
            self._on_status_update,
            10
        )
        
        # Wait for services
        while not self.execute_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('Waiting for execute_dance_command service...')
        
        self.get_logger().info('Dance command tester ready')
        
    def execute_command(self, command_name: str, duration: float = -1.0):
        """Execute a dance command"""
        request = ExecuteSingleCommand.Request()
        request.command_name = command_name
        request.expected_duration = duration  # -1.0 = use config default
        
        if duration > 0:
            self.get_logger().info(f'Executing command: {command_name} (override duration: {duration}s)')
        else:
            self.get_logger().info(f'Executing command: {command_name} (using config duration)')
        
        future = self.execute_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Command started: {response.message}')
                return response.estimated_duration
            else:
                self.get_logger().error(f'Command failed: {response.message}')
                return 0
        else:
            self.get_logger().error('Service call failed')
            return 0
            
    def stop_command(self):
        """Stop current command"""
        request = StopDanceRoutine.Request()
        
        future = self.stop_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f'Stop result: {response.message}')
            return response.success
        else:
            self.get_logger().error('Stop service call failed')
            return False
            
    def _on_status_update(self, msg: CommandExecutionStatus):
        """Handle status updates"""
        self.get_logger().info(
            f'Status: {msg.command_name} - '
            f'Status={msg.status}, Progress={msg.progress_percent:.1f}%, '
            f'Elapsed={msg.elapsed_time:.2f}s, Reason={msg.completion_reason}'
        )


def main():
    """Main test function"""
    rclpy.init()
    
    tester = DanceCommandTester()
    
    # Test commands - mix of config-based and override durations
    test_commands = [
        ("Hello", -1.0),      # Use config duration (should be 3.0s)
        ("Dance1", -1.0),     # Use config duration (should be 10.0s)
        ("FrontFlip", 3.0),   # Override config (normally 5.0s, use 3.0s)
        ("WiggleHips", -1.0), # Use config duration (should be 4.0s)
    ]
    
    try:
        for command_name, duration in test_commands:
            print(f"\n=== Testing {command_name} ===")
            
            # Execute command
            actual_duration = tester.execute_command(command_name, duration)
            if actual_duration > 0:
                # Wait for completion (with timeout)
                start_time = time.time()
                timeout = actual_duration * 2  # Double the actual duration
                
                print(f"Waiting up to {timeout}s for completion...")
                while time.time() - start_time < timeout:
                    rclpy.spin_once(tester, timeout_sec=0.1)
                    time.sleep(0.1)
                
                print(f"Finished testing {command_name}")
                time.sleep(2)  # Brief pause between commands
            else:
                print(f"Failed to start {command_name}")
                
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
        tester.stop_command()
        
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()