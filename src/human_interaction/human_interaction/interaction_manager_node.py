#!/usr/bin/env python3
"""
Interaction Manager Node

This node manages human interactions and orchestrates the behavior state machine.
It receives human detection data and coordinates appropriate responses including
gestures, speech, and navigation behaviors.

Subscribes to:
- /human_detection/people: Human detection results
- /human_detection/gestures: Recognized gestures
- /human_detection/proximity: Distance and zone information

Publishes to:
- /interaction/state: Current interaction state
- /interaction/commands: Commands for robot behaviors
- /interaction/events: Interaction events and triggers
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String
from geometry_msgs.msg import PoseArray
import json
import time
import threading
from typing import Dict, List, Optional
from enum import Enum

class InteractionState(Enum):
    """Interaction state enumeration"""
    PATROL = "patrol"
    HUMAN_DETECTED = "human_detected"
    INTERACTION = "interaction"
    COMMAND_EXECUTION = "command_execution"
    RESUME_PATROL = "resume_patrol"

class InteractionManagerNode(Node):
    """Interaction management and behavior orchestration node"""
    
    def __init__(self):
        super().__init__('interaction_manager_node')
        
        # Initialize parameters
        self.setup_parameters()
        
        # Initialize state
        self.current_state = InteractionState.PATROL
        self.last_state_change = time.time()
        self.interaction_start_time = None
        self.last_human_detection = None
        self.detected_humans = []
        self.active_gestures = []
        
        # State machine parameters
        self.state_timeout = self.get_parameter('state_timeout').value
        self.max_interaction_time = self.get_parameter('max_interaction_time').value
        self.resume_delay = self.get_parameter('resume_patrol_delay').value
        
        # Setup publishers
        self.setup_publishers()
        
        # Setup subscribers
        self.setup_subscribers()
        
        # Setup timers
        self.setup_timers()
        
        # Threading
        self.state_lock = threading.Lock()
        
        self.get_logger().info('Interaction Manager Node initialized')
    
    def setup_parameters(self):
        """Setup node parameters"""
        self.declare_parameter('state_timeout', 30.0)  # seconds
        self.declare_parameter('max_interaction_time', 60.0)  # seconds
        self.declare_parameter('resume_patrol_delay', 5.0)  # seconds
        self.declare_parameter('interaction_zone_distance', 2.0)  # meters
        self.declare_parameter('approach_zone_distance', 5.0)  # meters
        self.declare_parameter('enable_automatic_resume', True)
        self.declare_parameter('enable_gesture_responses', True)
        self.declare_parameter('enable_speech_responses', True)
    
    def setup_publishers(self):
        """Setup ROS publishers"""
        # Interaction state
        self.state_pub = self.create_publisher(
            String, 
            '/interaction/state', 
            10
        )
        
        # Behavior commands
        self.commands_pub = self.create_publisher(
            String, 
            '/interaction/commands', 
            10
        )
        
        # Interaction events
        self.events_pub = self.create_publisher(
            String, 
            '/interaction/events', 
            10
        )
        
        # TTS requests
        self.tts_pub = self.create_publisher(
            String, 
            '/tts', 
            10
        )
    
    def setup_subscribers(self):
        """Setup ROS subscribers"""
        # Human detection results
        self.people_sub = self.create_subscription(
            String,
            '/human_detection/people',
            self.people_callback,
            10
        )
        
        # Gesture recognition results
        self.gestures_sub = self.create_subscription(
            String,
            '/human_detection/gestures',
            self.gestures_callback,
            10
        )
        
        # Proximity information
        self.proximity_sub = self.create_subscription(
            String,
            '/human_detection/proximity',
            self.proximity_callback,
            10
        )
    
    def setup_timers(self):
        """Setup ROS timers"""
        # State machine timer
        self.state_timer = self.create_timer(
            1.0,  # 1 Hz
            self.state_machine_timer_callback
        )
        
        # Interaction timeout timer
        self.interaction_timer = self.create_timer(
            5.0,  # 5 Hz
            self.interaction_timeout_callback
        )
    
    def people_callback(self, msg: String):
        """Handle human detection results"""
        try:
            data = json.loads(msg.data)
            self.last_human_detection = data
            
            # Update detected humans list
            with self.state_lock:
                self.update_detected_humans(data)
            
            # Trigger state machine
            self.process_human_detection(data)
            
        except Exception as e:
            self.get_logger().error(f'Error processing people data: {e}')
    
    def gestures_callback(self, msg: String):
        """Handle gesture recognition results"""
        try:
            data = json.loads(msg.data)
            
            with self.state_lock:
                self.active_gestures = data.get('gestures', [])
            
            # Process gestures for interaction
            self.process_gestures(data)
            
        except Exception as e:
            self.get_logger().error(f'Error processing gestures: {e}')
    
    def proximity_callback(self, msg: String):
        """Handle proximity information"""
        try:
            data = json.loads(msg.data)
            
            # Update proximity-based behaviors
            self.process_proximity(data)
            
        except Exception as e:
            self.get_logger().error(f'Error processing proximity: {e}')
    
    def update_detected_humans(self, detection_data: Dict):
        """Update the list of detected humans"""
        timestamp = detection_data.get('timestamp', time.time())
        detection = detection_data.get('detection', {})
        distance = detection_data.get('distance', float('inf'))
        zone = detection_data.get('zone', 'far')
        
        # Add or update human in the list
        human_id = detection.get('id', 0)
        
        # Find existing human or add new one
        existing_human = None
        for human in self.detected_humans:
            if human.get('id') == human_id:
                existing_human = human
                break
        
        if existing_human:
            # Update existing human
            existing_human.update({
                'timestamp': timestamp,
                'distance': distance,
                'zone': zone,
                'confidence': detection.get('confidence', 0.0),
                'bbox': detection.get('bbox', [])
            })
        else:
            # Add new human
            self.detected_humans.append({
                'id': human_id,
                'timestamp': timestamp,
                'distance': distance,
                'zone': zone,
                'confidence': detection.get('confidence', 0.0),
                'bbox': detection.get('bbox', [])
            })
        
        # Remove old detections (older than 5 seconds)
        current_time = time.time()
        self.detected_humans = [
            human for human in self.detected_humans
            if current_time - human['timestamp'] < 5.0
        ]
    
    def process_human_detection(self, detection_data: Dict):
        """Process human detection and trigger state changes"""
        zone = detection_data.get('zone', 'far')
        distance = detection_data.get('distance', float('inf'))
        
        with self.state_lock:
            if self.current_state == InteractionState.PATROL:
                if zone == 'approach' or zone == 'interaction':
                    self.transition_to_state(InteractionState.HUMAN_DETECTED)
            
            elif self.current_state == InteractionState.HUMAN_DETECTED:
                if zone == 'interaction':
                    self.transition_to_state(InteractionState.INTERACTION)
                elif zone == 'far':
                    self.transition_to_state(InteractionState.RESUME_PATROL)
            
            elif self.current_state == InteractionState.INTERACTION:
                if zone == 'far':
                    self.transition_to_state(InteractionState.RESUME_PATROL)
    
    def process_gestures(self, gesture_data: Dict):
        """Process recognized gestures and trigger responses"""
        gestures = gesture_data.get('gestures', [])
        timestamp = gesture_data.get('timestamp', time.time())
        
        if not gestures:
            return
        
        with self.state_lock:
            if self.current_state == InteractionState.INTERACTION:
                # Generate gesture responses
                for gesture in gestures:
                    response = self.generate_gesture_response(gesture)
                    if response:
                        self.send_behavior_command(response)
    
    def process_proximity(self, proximity_data: Dict):
        """Process proximity information and adjust behaviors"""
        zone = proximity_data.get('zone', 'far')
        distance = proximity_data.get('distance', float('inf'))
        
        with self.state_lock:
            if self.current_state == InteractionState.INTERACTION:
                # Adjust behavior based on proximity
                if distance < 1.0:  # Very close
                    self.send_behavior_command('step_back')
                elif distance > 3.0:  # Too far
                    self.send_behavior_command('approach_human')
    
    def transition_to_state(self, new_state: InteractionState):
        """Transition to a new interaction state"""
        if new_state == self.current_state:
            return
        
        old_state = self.current_state
        self.current_state = new_state
        self.last_state_change = time.time()
        
        # Handle state-specific actions
        if new_state == InteractionState.INTERACTION:
            self.interaction_start_time = time.time()
            self.start_interaction()
        elif new_state == InteractionState.RESUME_PATROL:
            self.end_interaction()
        
        # Publish state change
        self.publish_state_change(old_state, new_state)
        
        self.get_logger().info(f'State transition: {old_state.value} -> {new_state.value}')
    
    def start_interaction(self):
        """Start human interaction sequence"""
        # Send greeting behavior
        self.send_behavior_command('greeting')
        
        # Send greeting speech
        self.send_speech("Hello! I'm your robot dog friend!")
        
        # Publish interaction event
        self.publish_interaction_event('interaction_started')
    
    def end_interaction(self):
        """End human interaction sequence"""
        # Send farewell behavior
        self.send_behavior_command('farewell')
        
        # Send farewell speech
        self.send_speech("See you later! Come back soon!")
        
        # Publish interaction event
        self.publish_interaction_event('interaction_ended')
        
        # Clear interaction state
        self.interaction_start_time = None
        self.detected_humans = []
        self.active_gestures = []
    
    def generate_gesture_response(self, gesture: str) -> Optional[str]:
        """Generate appropriate robot response to human gesture"""
        gesture_responses = {
            'wave': 'wave_back',
            'point': 'look_at_point',
            'thumbs_up': 'happy_dance',
            'peace_sign': 'peace_response',
            'hand_detected': 'hello_gesture'
        }
        
        return gesture_responses.get(gesture, None)
    
    def send_behavior_command(self, command: str):
        """Send behavior command to robot"""
        try:
            command_msg = String()
            command_data = {
                'command': command,
                'timestamp': time.time(),
                'priority': 'normal'
            }
            command_msg.data = json.dumps(command_data)
            self.commands_pub.publish(command_msg)
            
            self.get_logger().debug(f'Sent behavior command: {command}')
            
        except Exception as e:
            self.get_logger().error(f'Error sending behavior command: {e}')
    
    def send_speech(self, text: str):
        """Send text-to-speech request"""
        try:
            speech_msg = String()
            speech_msg.data = text
            self.tts_pub.publish(speech_msg)
            
            self.get_logger().debug(f'Sent speech: {text}')
            
        except Exception as e:
            self.get_logger().error(f'Error sending speech: {e}')
    
    def publish_state_change(self, old_state: InteractionState, new_state: InteractionState):
        """Publish state change event"""
        try:
            state_msg = String()
            state_data = {
                'current_state': new_state.value,
                'previous_state': old_state.value,
                'timestamp': time.time(),
                'detected_humans': len(self.detected_humans),
                'active_gestures': self.active_gestures
            }
            state_msg.data = json.dumps(state_data)
            self.state_pub.publish(state_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error publishing state change: {e}')
    
    def publish_interaction_event(self, event_type: str):
        """Publish interaction event"""
        try:
            event_msg = String()
            event_data = {
                'event_type': event_type,
                'timestamp': time.time(),
                'state': self.current_state.value
            }
            event_msg.data = json.dumps(event_data)
            self.events_pub.publish(event_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error publishing interaction event: {e}')
    
    def state_machine_timer_callback(self):
        """Timer callback for state machine logic"""
        current_time = time.time()
        
        with self.state_lock:
            # Check for state timeouts
            if current_time - self.last_state_change > self.state_timeout:
                if self.current_state == InteractionState.HUMAN_DETECTED:
                    self.transition_to_state(InteractionState.RESUME_PATROL)
            
            # Check for interaction timeout
            if (self.current_state == InteractionState.INTERACTION and 
                self.interaction_start_time and 
                current_time - self.interaction_start_time > self.max_interaction_time):
                self.transition_to_state(InteractionState.RESUME_PATROL)
    
    def interaction_timeout_callback(self):
        """Timer callback for interaction timeout checks"""
        current_time = time.time()
        
        with self.state_lock:
            # Check if humans are still detected
            if self.detected_humans:
                # Remove stale detections
                self.detected_humans = [
                    human for human in self.detected_humans
                    if current_time - human['timestamp'] < 3.0
                ]
            
            # If no humans detected and in interaction state, resume patrol
            if (not self.detected_humans and 
                self.current_state == InteractionState.INTERACTION):
                self.transition_to_state(InteractionState.RESUME_PATROL)


def main(args=None):
    rclpy.init(args=args)
    
    node = InteractionManagerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 