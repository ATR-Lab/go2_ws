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
from go2_interfaces.msg import WebRtcReq
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

class HumanInteractionState(Enum):
    """Per-human interaction state enumeration"""
    UNKNOWN = "unknown"
    APPROACHING = "approaching"
    FIRST_CONTACT = "first_contact"
    ACTIVE_INTERACTION = "active_interaction"
    IDLE_PRESENCE = "idle_presence"
    LEAVING = "leaving"
    GONE = "gone"

class CommandPriority(Enum):
    """Command priority levels"""
    SAFETY = 1      # step_back, emergency - always execute
    STATE_TRANSITION = 2  # greeting, farewell - once per state change
    GESTURE_RESPONSE = 3  # fist_bump, peace_sign - rate limited
    MAINTENANCE = 4       # patrol, idle - background only

# Robot command constants (from Go2 SDK)
ROBOT_COMMANDS = {
    # Basic Actions
    "Hello": 1016,
    "Sit": 1009,
    "StandUp": 1004,
    "Stretch": 1017,
    
    # Entertainment
    "Dance1": 1022,
    "Dance2": 1023,
    "WiggleHips": 1033,
    "FingerHeart": 1036,
    
    # Athletic
    "FrontFlip": 1030,
    "FrontJump": 1031,
    "MoonWalk": 1305,
    "Handstand": 1301,
}

# Gesture to robot command mapping (using actual detected gesture names)
GESTURE_COMMAND_MAP = {
    "right_fist": "Hello",
    "left_fist": "Hello",
    "right_open_hand": "Stretch", 
    "left_open_hand": "Stretch",
    "hands_visible": "Hello",
    "right_thumbs_up": "FingerHeart",
    "left_thumbs_up": "FingerHeart",
    "right_peace_sign": "Dance1",
    "left_peace_sign": "Dance1",
    "right_pointing": "Dance2",
    "left_pointing": "Dance2",
}

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
        
        # Multi-human state management
        self.detected_humans = {}  # Dict[human_id, human_state_data]
        self.active_interaction_human = None  # ID of human currently being interacted with
        self.interaction_queue = []  # Queue of humans waiting for interaction
        self.active_gestures = {}  # Dict[human_id, List[gestures]]
        
        # Smart interaction state management
        self.human_states = {}  # Dict[human_id, HumanInteractionState]
        self.human_state_timestamps = {}  # Dict[human_id, timestamp_of_last_state_change]
        self.human_last_interaction = {}  # Dict[human_id, timestamp_of_last_command]
        self.human_distance_history = {}  # Dict[human_id, List[(distance, timestamp)]]
        
        # Command coordination system
        self.command_queue = []  # List of pending commands with priority
        self.last_command_per_human = {}  # Dict[human_id, (command, timestamp)]
        self.command_cooldown_per_human = 3.0  # seconds between commands to same human
        self.state_transition_cooldown = 5.0  # seconds between state-based commands
        
        # Human interaction memory system
        self.human_interaction_history = {}  # Dict[human_id, interaction_history]
        
        # Priority and interaction management
        self.max_simultaneous_interactions = 1  # Only interact with one human at a time
        self.interaction_cooldown = {}  # Dict[human_id, last_interaction_time]
        self.human_timeout = 5.0  # seconds
        
        # Command rate limiting
        self.last_command_time = {}  # Dict[command_type, timestamp]
        self.command_rate_limit = 1.0  # seconds between same commands
        self.proximity_command_rate_limit = 2.0  # seconds between proximity commands
        self.last_proximity_command = 0.0
        
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
        
        # Robot commands (direct to Go2 driver)
        self.robot_command_pub = self.create_publisher(
            WebRtcReq,
            '/webrtc_req',
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
        
        # Combined gestures for multi-human management
        self.combined_gestures_sub = self.create_subscription(
            String,
            '/human_detection/combined_gestures',
            self.combined_gestures_callback,
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
        
        # Interaction transition timer
        self.transition_timer = self.create_timer(
            2.0,  # 2 Hz
            self.check_interaction_transitions
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
            human_id = data.get('human_id', 0)
            gestures = data.get('gestures', [])
            
            with self.state_lock:
                # Fix: Store gestures per human_id, not replace entire dictionary
                self.active_gestures[human_id] = gestures
            
            # Process gestures for interaction
            self.process_human_gestures(data)
            
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
    
    def combined_gestures_callback(self, msg: String):
        """Handle combined gestures from multiple humans with priority"""
        try:
            data = json.loads(msg.data)
            
            # Process prioritized gestures
            self.process_prioritized_gestures(data)
            
        except Exception as e:
            self.get_logger().error(f'Error processing combined gestures: {e}')
    
    def update_detected_humans(self, detection_data: Dict):
        """Update the dictionary of detected humans"""
        timestamp = detection_data.get('timestamp', time.time())
        detection = detection_data.get('detection', {})
        distance = detection_data.get('distance', float('inf'))
        zone = detection_data.get('zone', 'far')
        
        # Get human ID
        human_id = detection.get('id', 0)
        
        # Update or add human data
        self.detected_humans[human_id] = {
            'id': human_id,
            'timestamp': timestamp,
            'distance': distance,
            'zone': zone,
            'confidence': detection.get('confidence', 0.0),
            'bbox': detection.get('bbox', []),
            'last_seen': timestamp
        }
        
        # Remove old detections (older than human_timeout seconds)
        current_time = time.time()
        expired_humans = []
        for human_id, human_data in self.detected_humans.items():
            if current_time - human_data['timestamp'] > self.human_timeout:
                expired_humans.append(human_id)
        
        for human_id in expired_humans:
            del self.detected_humans[human_id]
            # Clean up related data
            if human_id in self.active_gestures:
                del self.active_gestures[human_id]
            if human_id == self.active_interaction_human:
                self.active_interaction_human = None
            if human_id in self.interaction_queue:
                self.interaction_queue.remove(human_id)
    
    def process_prioritized_gestures(self, combined_data: Dict):
        """Process prioritized gestures from multiple humans"""
        try:
            prioritized_gestures = combined_data.get('prioritized_gestures', [])
            total_humans = combined_data.get('total_humans', 0)
            
            if not prioritized_gestures:
                return
            
            with self.state_lock:
                # If no active interaction, start with highest priority human
                if self.active_interaction_human is None and prioritized_gestures:
                    highest_priority = prioritized_gestures[0]
                    human_id = highest_priority['human_id']
                    
                    # Check if this human is in interaction zone
                    if human_id in self.detected_humans:
                        human_data = self.detected_humans[human_id]
                        if human_data.get('zone') == 'interaction':
                            self.start_interaction_with_human(human_id)
                            self.process_human_gestures(highest_priority)
                
                # If we have an active interaction, only process gestures from that human
                elif self.active_interaction_human is not None:
                    for gesture_data in prioritized_gestures:
                        if gesture_data['human_id'] == self.active_interaction_human:
                            self.process_human_gestures(gesture_data)
                            break
                
                # Update interaction queue with remaining humans
                self.update_interaction_queue(prioritized_gestures)
                
        except Exception as e:
            self.get_logger().error(f'Error processing prioritized gestures: {e}')
    
    def start_interaction_with_human(self, human_id: int):
        """Start interaction with a specific human"""
        try:
            self.active_interaction_human = human_id
            self.interaction_start_time = time.time()
            
            # Transition to interaction state
            if self.current_state != InteractionState.INTERACTION:
                self.transition_to_state(InteractionState.INTERACTION)
            
            # Log interaction start
            self.get_logger().info(f'Started interaction with human {human_id}')
            
            # Publish interaction event
            event_msg = String()
            event_data = {
                'event': 'interaction_started',
                'human_id': human_id,
                'timestamp': time.time()
            }
            event_msg.data = json.dumps(event_data)
            self.events_pub.publish(event_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error starting interaction with human {human_id}: {e}')
    
    def send_robot_command(self, command_name: str, human_id: int = None) -> bool:
        """Send robot command directly to Go2 driver (same pattern as Go2 UI)"""
        try:
            if command_name not in ROBOT_COMMANDS:
                self.get_logger().error(f"Unknown robot command: {command_name}")
                return False
            
            # Create WebRTC request message
            req = WebRtcReq()
            req.id = 0  # Auto-assigned
            req.api_id = ROBOT_COMMANDS[command_name]
            req.topic = "rt/api/sport/request"  # Standard sport mode topic
            req.parameter = str(req.api_id)
            req.priority = 0  # Normal priority
            
            # Publish the request
            self.robot_command_pub.publish(req)
            
            # Log the command
            human_info = f" for human {human_id}" if human_id else ""
            self.get_logger().info(f'Robot command sent: {command_name} (API ID: {req.api_id}){human_info}')
            
            # Publish interaction event
            event_msg = String()
            event_data = {
                'event': 'robot_command_sent',
                'command': command_name,
                'human_id': human_id,
                'timestamp': time.time()
            }
            event_msg.data = json.dumps(event_data)
            self.events_pub.publish(event_msg)
            
            return True
            
        except Exception as e:
            self.get_logger().error(f'Error sending robot command {command_name}: {e}')
            return False
    
    def _map_behavior_to_robot_command(self, behavior_command: str) -> Optional[str]:
        """Map behavior command to robot command"""
        # Map behavior commands to robot commands
        behavior_to_robot_map = {
            'hello_gesture': 'Hello',
            'wave_back': 'Hello',
            'fist_bump': 'Hello',
            'happy_dance': 'Dance1',
            'peace_response': 'FingerHeart',
            'acknowledge': 'Hello',
            'look_at_point': 'Stretch',
        }
        
        return behavior_to_robot_map.get(behavior_command)
    
    def process_human_gestures(self, gesture_data: Dict):
        """Process gestures with smart context-aware logic"""
        try:
            human_id = gesture_data['human_id']
            gestures = gesture_data.get('gestures', [])
            gesture_states = gesture_data.get('gesture_states', {})
            
            if not gestures:
                return
            
            # Update active gestures for this human
            self.active_gestures[human_id] = gestures
            
            # Get human context
            human_data = self.detected_humans.get(human_id, {})
            distance = human_data.get('distance', float('inf'))
            zone = human_data.get('zone', 'far')
            
            # Update human interaction state based on proximity and movement
            self.update_human_interaction_state(human_id, distance, zone)
            
            # Process gestures with smart logic
            for gesture in gestures:
                gesture_state = gesture_states.get(gesture, 'unknown')
                
                # Only respond to NEW gestures
                if gesture_state == 'new':
                    response = self.smart_gesture_response(human_id, gesture, distance, zone)
                    if response:
                        self.queue_command(human_id, response, CommandPriority.GESTURE_RESPONSE)
                        self.get_logger().info(f'Smart response to "{gesture}" from human {human_id}: {response}')
                    else:
                        # Check for direct gesture-to-robot command mapping
                        if gesture in GESTURE_COMMAND_MAP:
                            robot_command = GESTURE_COMMAND_MAP[gesture]
                            self.send_robot_command(robot_command, human_id)
                            self.get_logger().info(f'Direct robot command "{robot_command}" for gesture "{gesture}" from human {human_id}')
                        else:
                            self.get_logger().debug(f'Ignoring gesture "{gesture}" from human {human_id} (context: {self.human_states.get(human_id, "unknown")})')
                        
        except Exception as e:
            import traceback
            self.get_logger().error(f'Error processing human gestures: {e}')
            self.get_logger().error(f'Traceback: {traceback.format_exc()}')
            self.get_logger().error(f'Gesture data: {gesture_data}')
    
    def update_interaction_queue(self, prioritized_gestures: List[Dict]):
        """Update the interaction queue based on priority"""
        try:
            # Clear current queue
            self.interaction_queue = []
            
            # Add humans with gestures to queue (excluding active interaction)
            for gesture_data in prioritized_gestures:
                human_id = gesture_data['human_id']
                if (human_id != self.active_interaction_human and 
                    human_id in self.detected_humans and
                    self.detected_humans[human_id].get('zone') in ['interaction', 'approach']):
                    self.interaction_queue.append(human_id)
                    
        except Exception as e:
            self.get_logger().error(f'Error updating interaction queue: {e}')
    
    def check_interaction_transitions(self):
        """Check if we need to transition between human interactions"""
        try:
            current_time = time.time()
            
            with self.state_lock:
                # Check if current interaction should end
                if self.active_interaction_human is not None:
                    # Check if human is still present and in interaction zone
                    if self.active_interaction_human not in self.detected_humans:
                        self.get_logger().info(f'Human {self.active_interaction_human} disappeared, ending interaction')
                        self.end_interaction()
                        return
                    
                    human_data = self.detected_humans[self.active_interaction_human]
                    if human_data.get('zone') == 'far':
                        self.get_logger().info(f'Human {self.active_interaction_human} moved away, ending interaction')
                        self.end_interaction()
                        return
                    
                    # Check interaction timeout
                    if (self.interaction_start_time and 
                        current_time - self.interaction_start_time > self.max_interaction_time):
                        self.get_logger().info(f'Interaction timeout with human {self.active_interaction_human}')
                        self.end_interaction()
                        return
                
                # Check if we should start a new interaction from the queue
                if (self.active_interaction_human is None and 
                    self.interaction_queue and 
                    self.current_state == InteractionState.PATROL):
                    
                    next_human_id = self.interaction_queue[0]
                    if (next_human_id in self.detected_humans and
                        self.detected_humans[next_human_id].get('zone') == 'interaction'):
                        self.start_interaction_with_human(next_human_id)
                        
        except Exception as e:
            self.get_logger().error(f'Error checking interaction transitions: {e}')
    
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
        """Process proximity information with smart command coordination"""
        zone = proximity_data.get('zone', 'far')
        distance = proximity_data.get('distance', float('inf'))
        
        with self.state_lock:
            # Find which human this proximity data belongs to (assume closest human for now)
            closest_human_id = None
            closest_distance = float('inf')
            
            for human_id, human_data in self.detected_humans.items():
                human_distance = human_data.get('distance', float('inf'))
                if human_distance < closest_distance:
                    closest_distance = human_distance
                    closest_human_id = human_id
            
            if closest_human_id and self.current_state == InteractionState.INTERACTION:
                # Safety-first proximity handling
                if distance < 0.8:  # Too close - safety priority
                    self.queue_command(closest_human_id, 'step_back', CommandPriority.SAFETY)
                elif distance > 4.0 and zone == 'far':  # Too far - approach if was interacting
                    human_state = self.human_states.get(closest_human_id, HumanInteractionState.UNKNOWN)
                    if human_state in [HumanInteractionState.ACTIVE_INTERACTION, HumanInteractionState.IDLE_PRESENCE]:
                        self.queue_command(closest_human_id, 'approach_human', CommandPriority.GESTURE_RESPONSE)
    
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
        # Note: Farewell is now handled by smart state machine, not here
        # This prevents double farewell commands
        
        # Send farewell speech
        self.send_speech("See you later! Come back soon!")
        
        # Publish interaction event
        self.publish_interaction_event('interaction_ended')
        
        # Clear interaction state
        self.interaction_start_time = None
        self.active_interaction_human = None
        self.interaction_queue = []
        # Note: Don't clear detected_humans as they may still be present
        # Only clear active_gestures for the interaction that ended
        if hasattr(self, 'active_interaction_human') and self.active_interaction_human in self.active_gestures:
            del self.active_gestures[self.active_interaction_human]
    
    def generate_gesture_response(self, gesture: str, human_id: int = None) -> Optional[str]:
        """Generate appropriate robot response to human gesture"""
        gesture_responses = {
            'left_wave': 'wave_back',
            'right_wave': 'wave_back',
            'left_pointing': 'look_at_point',
            'right_pointing': 'look_at_point',
            'left_thumbs_up': 'happy_dance',
            'right_thumbs_up': 'happy_dance',
            'left_peace_sign': 'peace_response',
            'right_peace_sign': 'peace_response',
            'left_ok_sign': 'acknowledge',
            'right_ok_sign': 'acknowledge',
            'left_fist': 'fist_bump',
            'right_fist': 'fist_bump',
            'hands_visible': 'hello_gesture',
            # Legacy support
            'wave': 'wave_back',
            'point': 'look_at_point',
            'thumbs_up': 'happy_dance',
            'peace_sign': 'peace_response',
            'hand_detected': 'hello_gesture'
        }
        
        response = gesture_responses.get(gesture, None)
        
        # Log gesture response with human ID
        if response and human_id is not None:
            self.get_logger().info(f'Responding to gesture "{gesture}" from human {human_id} with "{response}"')
        
        return response
    
    def send_behavior_command(self, command: str):
        """Send behavior command to robot (legacy method - calls rate limited version)"""
        current_time = time.time()
        self.send_behavior_command_with_rate_limit(command, current_time)
    
    def send_behavior_command_with_rate_limit(self, command: str, current_time: float) -> bool:
        """Send behavior command to robot with rate limiting"""
        try:
            # Check rate limiting
            if command in self.last_command_time:
                time_since_last = current_time - self.last_command_time[command]
                if time_since_last < self.command_rate_limit:
                    self.get_logger().debug(f'Rate limiting command: {command} (last sent {time_since_last:.1f}s ago)')
                    return False  # Command was rate limited
            
            # Send command
            command_msg = String()
            command_data = {
                'command': command,
                'timestamp': current_time,
                'priority': 'normal'
            }
            command_msg.data = json.dumps(command_data)
            self.commands_pub.publish(command_msg)
            
            # Check if this command maps to a robot gesture and send robot command
            robot_command = self._map_behavior_to_robot_command(command)
            if robot_command:
                self.send_robot_command(robot_command)
            
            # Update rate limiting tracker
            self.last_command_time[command] = current_time
            
            self.get_logger().debug(f'Sent behavior command: {command}')
            return True
            
        except Exception as e:
            self.get_logger().error(f'Error sending behavior command: {e}')
            return False
    
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
                # Remove stale detections (now using dictionary format)
                expired_humans = []
                for human_id, human_data in self.detected_humans.items():
                    if current_time - human_data['timestamp'] > 3.0:
                        expired_humans.append(human_id)
                
                # Clean up expired humans
                for human_id in expired_humans:
                    del self.detected_humans[human_id]
                    # Clean up related data
                    if human_id in self.active_gestures:
                        del self.active_gestures[human_id]
                    if human_id == self.active_interaction_human:
                        self.active_interaction_human = None
                    if human_id in self.interaction_queue:
                        self.interaction_queue.remove(human_id)
            
            # If no humans detected and in interaction state, resume patrol
            if (not self.detected_humans and 
                self.current_state == InteractionState.INTERACTION):
                self.transition_to_state(InteractionState.RESUME_PATROL)


    def detect_movement_direction(self, human_id: int) -> str:
        """Detect if human is approaching, leaving, or stationary"""
        try:
            if human_id not in self.human_distance_history:
                return 'unknown'
            
            distance_history = self.human_distance_history[human_id]
            if len(distance_history) < 2:
                return 'unknown'
            
            # Compare recent distances to detect trend
            recent_distances = [d for d, t in distance_history[-3:]]
            
            if len(recent_distances) >= 2:
                # Calculate average change
                distance_change = recent_distances[-1] - recent_distances[0]
                
                if distance_change < -0.5:  # Getting closer
                    return 'approaching'
                elif distance_change > 0.5:  # Getting farther
                    return 'leaving'
                else:
                    return 'stationary'
            
            return 'unknown'
            
        except Exception as e:
            self.get_logger().error(f'Error detecting movement: {e}')
            return 'unknown'
    
    def update_interaction_state(self, human_id: int, zone: str, movement: str):
        """Update human interaction state based on proximity and movement"""
        try:
            current_state = self.human_interaction_states.get(human_id, 'unknown')
            new_state = current_state
            
            # State transition logic
            if zone == 'far':
                if movement == 'approaching':
                    new_state = 'approaching'
                else:
                    new_state = 'far_away'
            
            elif zone == 'approach':
                if movement == 'approaching':
                    new_state = 'approaching'
                elif movement == 'leaving':
                    new_state = 'leaving'
                else:
                    new_state = 'nearby'
            
            elif zone == 'interaction':
                if movement == 'leaving':
                    new_state = 'leaving'
                elif not self.human_greeting_status.get(human_id, False):
                    new_state = 'greeting_needed'
                else:
                    # Determine if actively interacting or just hanging out
                    current_time = time.time()
                    last_gesture_time = 0
                    
                    # Check when we last responded to this human
                    if human_id in self.interaction_cooldown:
                        last_gesture_time = self.interaction_cooldown[human_id]
                    
                    # If no recent interaction, consider them idle
                    if current_time - last_gesture_time > 30.0:  # 30 seconds
                        new_state = 'idle_together'
                    else:
                        new_state = 'conversing'
            
            # Update state if changed
            if new_state != current_state:
                self.human_interaction_states[human_id] = new_state
                self.get_logger().debug(f'Human {human_id} state: {current_state} → {new_state} (zone: {zone}, movement: {movement})')
                
                # Trigger farewell if leaving
                if new_state == 'leaving' and current_state in ['conversing', 'idle_together']:
                    self.send_behavior_command('farewell')
                    self.get_logger().info(f'Human {human_id} is leaving - sending farewell')
            
        except Exception as e:
            self.get_logger().error(f'Error updating interaction state: {e}')
    
    def update_human_interaction_state(self, human_id: int, distance: float, zone: str):
        """Update per-human interaction state based on proximity and context"""
        try:
            current_time = time.time()
            current_state = self.human_states.get(human_id, HumanInteractionState.UNKNOWN)
            new_state = current_state
            
            # Initialize if new human
            if human_id not in self.human_states:
                self.human_states[human_id] = HumanInteractionState.UNKNOWN
                self.human_state_timestamps[human_id] = current_time
                self.human_last_interaction[human_id] = 0
                
                # Initialize interaction history
                if human_id not in self.human_interaction_history:
                    self.human_interaction_history[human_id] = {
                        'first_seen': current_time,
                        'last_interaction': 0,
                        'total_interactions': 0,
                        'last_farewell': 0,
                        'interaction_type': 'first_time'
                    }
            
            # Update distance history
            if human_id not in self.human_distance_history:
                self.human_distance_history[human_id] = []
            
            distance_history = self.human_distance_history[human_id]
            distance_history.append((distance, current_time))
            
            # Keep only recent history (last 10 seconds)
            cutoff_time = current_time - 10.0
            distance_history[:] = [(d, t) for d, t in distance_history if t > cutoff_time]
            
            # Detect movement direction
            movement = self.detect_movement_direction(human_id)
            
            # State transition logic
            if zone == 'far':
                if movement == 'approaching':
                    new_state = HumanInteractionState.APPROACHING
                else:
                    new_state = HumanInteractionState.UNKNOWN
            
            elif zone == 'approach':
                if movement == 'approaching':
                    new_state = HumanInteractionState.APPROACHING
                elif movement == 'leaving':
                    new_state = HumanInteractionState.LEAVING
                else:
                    new_state = HumanInteractionState.APPROACHING  # Default for approach zone
            
            elif zone == 'interaction':
                if movement == 'leaving':
                    new_state = HumanInteractionState.LEAVING
                elif current_state == HumanInteractionState.UNKNOWN or current_state == HumanInteractionState.APPROACHING:
                    new_state = HumanInteractionState.FIRST_CONTACT
                elif current_state == HumanInteractionState.FIRST_CONTACT:
                    # Check if we've had recent interaction
                    last_interaction = self.human_last_interaction.get(human_id, 0)
                    if current_time - last_interaction < 10.0:
                        new_state = HumanInteractionState.ACTIVE_INTERACTION
                    else:
                        new_state = HumanInteractionState.FIRST_CONTACT  # Stay in first contact
                elif current_state == HumanInteractionState.ACTIVE_INTERACTION:
                    # Check if interaction has gone idle
                    last_interaction = self.human_last_interaction.get(human_id, 0)
                    if current_time - last_interaction > 30.0:
                        new_state = HumanInteractionState.IDLE_PRESENCE
                else:
                    new_state = current_state  # Maintain current state
            
            # Handle state transitions
            if new_state != current_state:
                self.human_states[human_id] = new_state
                self.human_state_timestamps[human_id] = current_time
                
                self.get_logger().info(f'Human {human_id} state: {current_state.value} → {new_state.value} (zone: {zone}, movement: {movement})')
                
                # Trigger state-based commands
                if new_state == HumanInteractionState.APPROACHING and current_state == HumanInteractionState.UNKNOWN:
                    # Someone is approaching - no immediate action, wait for gesture
                    pass
                elif new_state == HumanInteractionState.FIRST_CONTACT:
                    # Ready for first interaction - wait for gesture
                    pass
                elif new_state == HumanInteractionState.LEAVING:
                    # Send farewell if was interacting
                    if current_state in [HumanInteractionState.ACTIVE_INTERACTION, HumanInteractionState.IDLE_PRESENCE]:
                        self.queue_command(human_id, 'farewell', CommandPriority.STATE_TRANSITION)
                        # Record farewell in interaction history
                        self.human_interaction_history[human_id]['last_farewell'] = current_time
                        
        except Exception as e:
            self.get_logger().error(f'Error updating human interaction state: {e}')
    
    def smart_gesture_response(self, human_id: int, gesture: str, distance: float, zone: str) -> Optional[str]:
        """Generate smart context-aware response to gesture"""
        try:
            current_time = time.time()
            human_state = self.human_states.get(human_id, HumanInteractionState.UNKNOWN)
            last_interaction = self.human_last_interaction.get(human_id, 0)
            
            # Safety first - if too close, suppress gesture responses
            if distance < 0.8:
                return None  # Let safety commands handle this
            
            # Zone-based filtering
            if zone == 'far':
                return None  # Ignore gestures from far away
            
            elif zone == 'approach':
                # Only acknowledge waves when approaching
                if human_state == HumanInteractionState.APPROACHING:
                    if gesture in ['left_wave', 'right_wave', 'wave']:
                        return 'wave_back'  # Acknowledge approach
                return None
            
            elif zone == 'interaction':
                # Full gesture processing based on interaction state
                
                if human_state == HumanInteractionState.FIRST_CONTACT:
                    # First interaction - respond to greeting gestures with context awareness
                    if gesture in ['left_wave', 'right_wave', 'wave', 'hands_visible']:
                        self.human_last_interaction[human_id] = current_time
                        
                        # Update interaction history
                        history = self.human_interaction_history[human_id]
                        history['last_interaction'] = current_time
                        history['total_interactions'] += 1
                        
                        # Determine greeting type based on return status
                        return_type = self.detect_return_type(human_id, current_time)
                        greeting_responses = {
                            'first_time': 'hello_gesture',
                            'quick_return': 'wave_back',  # Welcome back!
                            'short_return': 'hello_gesture',  # Good to see you again
                            'long_return': 'hello_gesture'  # Hello again
                        }
                        
                        response = greeting_responses.get(return_type, 'hello_gesture')
                        self.get_logger().info(f'Greeting human {human_id} with {response} (return_type: {return_type})')
                        return response
                    return None  # Wait for proper greeting
                
                elif human_state == HumanInteractionState.ACTIVE_INTERACTION:
                    # Active conversation - check cooldown
                    if current_time - last_interaction < self.command_cooldown_per_human:
                        return None  # Too soon since last interaction
                    
                    # Full gesture responses
                    gesture_map = {
                        'left_fist': 'fist_bump', 'right_fist': 'fist_bump',
                        'left_thumbs_up': 'happy_dance', 'right_thumbs_up': 'happy_dance',
                        'left_peace_sign': 'peace_response', 'right_peace_sign': 'peace_response',
                        'left_pointing': 'look_at_point', 'right_pointing': 'look_at_point',
                        'left_ok_sign': 'acknowledge', 'right_ok_sign': 'acknowledge'
                    }
                    
                    response = gesture_map.get(gesture, None)
                    if response:
                        self.human_last_interaction[human_id] = current_time
                    return response
                
                elif human_state == HumanInteractionState.IDLE_PRESENCE:
                    # Idle together - only respond to attention-seeking gestures
                    attention_gestures = {
                        'left_thumbs_up': 'happy_dance', 'right_thumbs_up': 'happy_dance',
                        'left_peace_sign': 'peace_response', 'right_peace_sign': 'peace_response',
                        'left_fist': 'fist_bump', 'right_fist': 'fist_bump',
                        'left_pointing': 'look_at_point', 'right_pointing': 'look_at_point'
                    }
                    
                    if gesture in attention_gestures:
                        # Check cooldown for attention gestures
                        if current_time - last_interaction > 10.0:  # Longer cooldown for idle
                            self.human_last_interaction[human_id] = current_time
                            return attention_gestures[gesture]
                    
                    # Ignore casual gestures (wave, hands_visible) when idle
                    return None
            
            return None
            
        except Exception as e:
            self.get_logger().error(f'Error in smart gesture response: {e}')
            return None
    
    def queue_command(self, human_id: int, command: str, priority: CommandPriority):
        """Queue command with priority and coordination"""
        try:
            current_time = time.time()
            
            # Check if we can send command to this human
            if human_id in self.last_command_per_human:
                last_command, last_time = self.last_command_per_human[human_id]
                
                # Priority-based cooldown
                if priority == CommandPriority.SAFETY:
                    cooldown = 0.5  # Safety commands have minimal cooldown
                elif priority == CommandPriority.STATE_TRANSITION:
                    cooldown = self.state_transition_cooldown
                else:
                    cooldown = self.command_cooldown_per_human
                
                if current_time - last_time < cooldown:
                    self.get_logger().debug(f'Command {command} to human {human_id} blocked by cooldown')
                    return
            
            # Execute command immediately (for now - could implement actual queue later)
            self.send_behavior_command(command)
            self.last_command_per_human[human_id] = (command, current_time)
            
            self.get_logger().debug(f'Executed {priority.name} command "{command}" for human {human_id}')
            
        except Exception as e:
            self.get_logger().error(f'Error queuing command: {e}')
    
    def detect_return_type(self, human_id: int, current_time: float) -> str:
        """Detect if human is returning and what type of return it is"""
        try:
            if human_id not in self.human_interaction_history:
                return 'first_time'
            
            history = self.human_interaction_history[human_id]
            last_farewell = history.get('last_farewell', 0)
            total_interactions = history.get('total_interactions', 0)
            
            # If no previous farewell, this is first time
            if last_farewell == 0:
                return 'first_time'
            
            # Calculate time since last farewell
            time_away = current_time - last_farewell
            
            if time_away < 60:  # Less than 1 minute
                return 'quick_return'
            elif time_away < 300:  # Less than 5 minutes
                return 'short_return'
            elif total_interactions > 2:  # Frequent visitor
                return 'long_return'
            else:
                return 'first_time'  # Treat as new if long time and few interactions
                
        except Exception as e:
            self.get_logger().error(f'Error detecting return type: {e}')
            return 'first_time'


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