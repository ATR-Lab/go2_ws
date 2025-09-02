#!/usr/bin/env python3
"""
Navigation Coordinator Node

Ultra-simple Nav2 coordinator for human-aware navigation in the robot dog petting zoo.
Translates human interaction events into Nav2 action calls while letting Nav2 handle 
all the complex navigation logic.

Subscribes to:
- /interaction/nav_commands: Navigation commands from interaction_manager
- /interaction/events: Human presence events (entered/left area)
- /goal_pose: External navigation goals (iPad interface, patrol system)

Integrates with Nav2:
- NavigateToPose action client for goal management
- BackUp action client for step-back behaviors
- Uses Nav2's existing goal cancellation and management

Author: Robot Dog Petting Zoo Team
License: BSD-3-Clause
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose, BackUp
from action_msgs.msg import GoalStatus
import json
import time
from typing import Optional

class NavigationCoordinator(Node):
    """Ultra-simple navigation coordinator for human-aware navigation"""
    
    def __init__(self):
        super().__init__('navigation_coordinator')
        
        # Simple parameters
        self.declare_parameter('step_back_distance', 0.5)  # meters
        self.declare_parameter('step_back_speed', 0.2)     # m/s
        self.declare_parameter('human_timeout', 5.0)       # seconds before "human left"
        self.declare_parameter('resume_delay', 2.0)        # seconds delay before resuming
        
        self.step_back_distance = self.get_parameter('step_back_distance').value
        self.step_back_speed = self.get_parameter('step_back_speed').value
        self.human_timeout = self.get_parameter('human_timeout').value
        self.resume_delay = self.get_parameter('resume_delay').value
        
        # Simple state tracking
        self.stored_goal: Optional[PoseStamped] = None
        self.humans_present = False
        self.navigation_paused = False
        self.last_human_seen_time = 0.0
        
        # Nav2 action clients
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.backup_client = ActionClient(self, BackUp, 'backup')
        
        # Wait for Nav2 action servers
        self.get_logger().info('Waiting for Nav2 action servers...')
        if not self.nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().warn('NavigateToPose action server not available')
        else:
            self.get_logger().info('NavigateToPose action server ready')
            
        if not self.backup_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().warn('BackUp action server not available')
        else:
            self.get_logger().info('BackUp action server ready')
        
        # Subscribers
        self.setup_subscribers()
        
        # Timer for checking human timeout
        self.human_timeout_timer = self.create_timer(1.0, self.check_human_timeout)
        
        self.get_logger().info('Navigation Coordinator initialized (simplified)')
    
    def setup_subscribers(self):
        """Setup ROS subscribers"""
        # Navigation commands from interaction manager
        self.nav_commands_sub = self.create_subscription(
            String, '/interaction/nav_commands', self.nav_command_callback, 10
        )
        
        # Human presence events
        self.interaction_events_sub = self.create_subscription(
            String, '/interaction/events', self.interaction_events_callback, 10
        )
        
        # External navigation goals (iPad interface, patrol system)
        self.goal_pose_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_pose_callback, 10
        )
        
        # Human detection for presence tracking
        self.people_sub = self.create_subscription(
            String, '/human_detection/people', self.people_callback, 10
        )
    
    def people_callback(self, msg: String):
        """Track human presence in camera view"""
        try:
            people_data = json.loads(msg.data)
            people = people_data.get('people', {})
            
            current_time = time.time()
            
            if people:
                # Humans detected
                if not self.humans_present:
                    self.get_logger().info('Humans entered camera view - pausing navigation')
                    self.humans_present = True
                    self.pause_navigation_for_humans()
                
                self.last_human_seen_time = current_time
            else:
                # No humans detected - but don't immediately resume
                # Let the timeout timer handle this to avoid flicker
                pass
                
        except json.JSONDecodeError as e:
            self.get_logger().warn(f'Failed to parse people data: {e}')
    
    def check_human_timeout(self):
        """Check if humans have been gone long enough to resume navigation"""
        if self.humans_present:
            current_time = time.time()
            time_since_last_human = current_time - self.last_human_seen_time
            
            if time_since_last_human > self.human_timeout:
                self.get_logger().info('Humans left area - resuming navigation')
                self.humans_present = False
                self.resume_navigation_after_humans_left()
    
    def nav_command_callback(self, msg: String):
        """Handle navigation commands from interaction manager"""
        try:
            command_data = json.loads(msg.data)
            command = command_data.get('command')
            human_id = command_data.get('human_id')
            
            self.get_logger().info(f'Received navigation command: {command} for human {human_id}')
            
            # Execute navigation command
            if command == 'step_back':
                self.execute_step_back()
            elif command == 'pause_navigation':
                self.pause_navigation_for_humans()
            elif command == 'resume_navigation':
                self.resume_navigation_after_humans_left()
            else:
                self.get_logger().warn(f'Unknown navigation command: {command}')
                
        except json.JSONDecodeError as e:
            self.get_logger().warn(f'Failed to parse navigation command: {e}')
    
    def interaction_events_callback(self, msg: String):
        """Handle interaction events"""
        try:
            event_data = json.loads(msg.data)
            event_type = event_data.get('event')
            
            if event_type == 'human_entered_area':
                self.get_logger().info('Human entered area event received')
                self.humans_present = True
                self.pause_navigation_for_humans()
            elif event_type == 'human_left_area':
                self.get_logger().info('Human left area event received')
                self.humans_present = False
                self.resume_navigation_after_humans_left()
                
        except json.JSONDecodeError as e:
            self.get_logger().warn(f'Failed to parse interaction event: {e}')
    
    def goal_pose_callback(self, msg: PoseStamped):
        """Handle new goal poses (e.g., from iPad interface)"""
        if not self.humans_present:
            # Start navigation immediately if no humans present
            self.start_navigation_to_pose(msg)
        else:
            # Store goal for later if humans are present
            self.stored_goal = msg
            self.get_logger().info('Goal stored - will navigate after humans leave')
    
    def pause_navigation_for_humans(self):
        """Pause current navigation when humans are present"""
        if not self.navigation_paused:
            self.get_logger().info('Pausing navigation - humans present')
            self.nav_client.cancel_all_goals()
            self.navigation_paused = True
    
    def resume_navigation_after_humans_left(self):
        """Resume navigation after humans leave"""
        if self.navigation_paused and self.stored_goal:
            self.get_logger().info('Resuming navigation - humans left')
            
            # Add small delay to ensure humans are really gone
            self.create_timer(self.resume_delay, self.delayed_resume_navigation)
            
    def delayed_resume_navigation(self):
        """Resume navigation after delay"""
        if self.stored_goal and not self.humans_present:
            self.start_navigation_to_pose(self.stored_goal)
            self.navigation_paused = False
    
    def start_navigation_to_pose(self, goal_pose: PoseStamped):
        """Start navigation to a pose using Nav2"""
        if not self.nav_client.server_is_ready():
            self.get_logger().warn('NavigateToPose action server not ready')
            return False
            
        # Create navigation goal
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal_pose
        
        # Send goal
        self.get_logger().info(f'Starting navigation to pose: [{goal_pose.pose.position.x:.2f}, '
                              f'{goal_pose.pose.position.y:.2f}]')
        
        future = self.nav_client.send_goal_async(nav_goal)
        future.add_done_callback(self.nav_goal_response_callback)
        
        self.stored_goal = goal_pose
        return True
    
    def execute_step_back(self):
        """Execute step back behavior using Nav2 BackUp action"""
        if not self.backup_client.server_is_ready():
            self.get_logger().warn('BackUp action server not ready')
            return False
            
        self.get_logger().info(f'Executing step back: {self.step_back_distance}m at {self.step_back_speed}m/s')
        
        # Create backup goal
        backup_goal = BackUp.Goal()
        backup_goal.target = self.step_back_distance
        backup_goal.speed = self.step_back_speed
        
        # Send backup goal
        future = self.backup_client.send_goal_async(backup_goal)
        future.add_done_callback(self.backup_goal_response_callback)
        
        return True
    
    def nav_goal_response_callback(self, future):
        """Handle navigation goal response"""
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().warn('Navigation goal rejected')
                return
                
            self.get_logger().info('Navigation goal accepted')
            
            # Get result
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.nav_goal_result_callback)
            
        except Exception as e:
            self.get_logger().error(f'Error in navigation goal response: {e}')
    
    def nav_goal_result_callback(self, future):
        """Handle navigation goal result"""
        try:
            result = future.result()
            
            if result.status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info('Navigation goal succeeded')
                self.stored_goal = None
                
            elif result.status == GoalStatus.STATUS_CANCELED:
                self.get_logger().info('Navigation goal canceled')
                
            else:
                self.get_logger().warn(f'Navigation goal failed with status: {result.status}')
                
        except Exception as e:
            self.get_logger().error(f'Error in navigation goal result: {e}')
    
    def backup_goal_response_callback(self, future):
        """Handle backup goal response"""
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().warn('Step back goal rejected')
                return
                
            self.get_logger().info('Step back goal accepted')
            
            # Get result
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.backup_goal_result_callback)
            
        except Exception as e:
            self.get_logger().error(f'Error in backup goal response: {e}')
    
    def backup_goal_result_callback(self, future):
        """Handle backup goal result"""
        try:
            result = future.result()
            
            if result.status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info('Step back completed successfully')
            else:
                self.get_logger().warn(f'Step back failed with status: {result.status}')
                
        except Exception as e:
            self.get_logger().error(f'Error in backup goal result: {e}')

def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        node = NavigationCoordinator()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()