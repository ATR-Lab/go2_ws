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
        
    def execute_command(self, command_name: str):
        """Execute a dance command"""
        request = ExecuteSingleCommand.Request()
        request.command_name = command_name
        
        self.get_logger().info(f'Executing command: {command_name} (using robot state feedback)')
        
        future = self.execute_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Command started: {response.message}')
                return True
            else:
                self.get_logger().error(f'Command failed: {response.message}')
                return False
        else:
            self.get_logger().error('Service call failed')
            return False
            
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
    
    # Test commands using pure robot state feedback
    test_commands = [
        "Hello",
        "Dance1", 
        "FrontFlip",
        "WiggleHips",
    ]
    
    try:
        for command_name in test_commands:
            print(f"\n=== Testing {command_name} ===")
            
            # Execute command
            success = tester.execute_command(command_name)
            if success:
                # Wait for completion with generous timeout
                start_time = time.time()
                timeout = 60.0  # Universal 60s safety timeout
                
                print(f"Waiting for robot feedback completion (max {timeout}s)...")
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