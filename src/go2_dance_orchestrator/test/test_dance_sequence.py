#!/usr/bin/env python3

"""
Test script for dance sequence execution.
"""

import rclpy
import time
from rclpy.node import Node

from go2_interfaces.srv import StartDanceRoutine, StopDanceRoutine
from go2_interfaces.msg import DanceRoutineStatus


class DanceSequenceTester(Node):
    """Test client for dance sequence execution"""
    
    def __init__(self):
        super().__init__('dance_sequence_tester')
        
        # Service clients
        self.start_routine_client = self.create_client(
            StartDanceRoutine, 
            '/start_dance_routine'
        )
        
        self.stop_routine_client = self.create_client(
            StopDanceRoutine, 
            '/stop_dance_command'
        )
        
        # Status subscriber
        self.status_subscriber = self.create_subscription(
            DanceRoutineStatus,
            '/dance_routine_status',
            self.status_callback,
            10
        )
        
        self.last_status = None
        
    def status_callback(self, msg):
        """Handle routine status updates"""
        self.last_status = msg
        current_cmd = msg.current_command_name or "None"
        self.get_logger().info(
            f"Routine '{msg.routine_name}': "
            f"Command {msg.current_command_index + 1}/{msg.total_commands} ({current_cmd}) - "
            f"Routine: {msg.routine_progress_percent:.1f}%, "
            f"Current: {msg.current_command_progress_percent:.1f}%, "
            f"Status: {msg.status}"
        )
        
    def execute_routine(self, command_sequence: list, routine_name: str = ""):
        """Execute a dance routine"""
        request = StartDanceRoutine.Request()
        request.command_sequence = command_sequence
        request.routine_name = routine_name
        
        self.get_logger().info(f'Executing dance routine: {command_sequence}')
        
        # Wait for service
        if not self.start_routine_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Start dance routine service not available')
            return False
            
        # Call service
        future = self.start_routine_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info(f'✅ {response.message}')
                return True
            else:
                self.get_logger().error(f'❌ {response.message}')
                return False
        else:
            self.get_logger().error('Failed to call start dance routine service')
            return False
            
    def stop_routine(self):
        """Stop current routine"""
        request = StopDanceRoutine.Request()
        
        self.get_logger().info('Stopping current routine...')
        
        if not self.stop_routine_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Stop routine service not available')
            return False
            
        future = self.stop_routine_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f'Stop result: {response.message}')
            return response.success
        else:
            self.get_logger().error('Failed to call stop routine service')
            return False


def main():
    """Main test function"""
    rclpy.init()
    
    tester = DanceSequenceTester()
    
    # Test routines
    test_routines = [
        {
            "name": "greeting_sequence", 
            "commands": ["Hello", "FingerHeart"],
            "expected_duration": 10.0  # 3 + 3 seconds
        },
        {
            "name": "performance_routine",
            "commands": ["Hello", "Dance1", "FrontFlip"],
            "expected_duration": 28.0  # 3 + 20 + 5 seconds
        },
    ]
    
    try:
        for routine in test_routines:
            print(f"\n{'='*60}")
            print(f"🎭 Testing: {routine['name']}")
            print(f"Commands: {routine['commands']}")
            print(f"Expected duration: ~{routine['expected_duration']}s")
            print('='*60)
            
            # Execute routine
            success = tester.execute_routine(
                command_sequence=routine["commands"],
                routine_name=routine["name"]
            )
            
            if success:
                # Wait for completion with buffer
                start_time = time.time()
                timeout = routine["expected_duration"] + 5.0  # 5s buffer
                
                print(f"⏳ Waiting for routine completion (max {timeout:.0f}s)...")
                
                while time.time() - start_time < timeout:
                    rclpy.spin_once(tester, timeout_sec=0.1)
                    
                    if (tester.last_status and 
                        tester.last_status.status in ["COMPLETED", "FAILED", "CANCELLED"]):
                        print(f"✅ Routine completed with status: {tester.last_status.status}")
                        break
                        
                    time.sleep(0.1)
                else:
                    print(f"⏰ Timeout reached - routine may still be running")
                    
                # Brief pause between tests
                print("⏸️  Pausing before next test...")
                time.sleep(3)
                
            else:
                print(f"❌ Failed to start {routine['name']}")
                
        print(f"\n{'='*60}")
        print("🏁 All tests completed!")
        print('='*60)
        
    except KeyboardInterrupt:
        print("\n🛑 Test interrupted by user")
        tester.stop_routine()
    except Exception as e:
        print(f"\n💥 Test error: {e}")
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()