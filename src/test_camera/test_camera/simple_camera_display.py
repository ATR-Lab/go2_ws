#!/usr/bin/env python3
"""
Simple Camera Display Node

A straightforward camera display that shows the USB camera feed with
YOLO human detection and MediaPipe gesture recognition overlays.
Uses OpenCV's built-in display to avoid Qt conflicts.

Usage:
    ros2 run test_camera simple_camera_display
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import json
import time
from typing import Dict, List, Optional

class SimpleCameraDisplay(Node):
    """Simple camera display with YOLO and MediaPipe overlays"""
    
    def __init__(self):
        super().__init__('simple_camera_display')
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Camera state
        self.current_image = None
        self.current_detections = {}  # Dict[human_id, detection_data]
        self.current_gestures = {}   # Dict[human_id, gesture_data]
        self.combined_gestures = {}  # Priority-ordered gestures
        self.current_state = "Unknown"
        self.interaction_events = []  # Recent interaction events
        
        # Performance tracking
        self.frame_count = 0
        self.last_fps_time = time.time()
        self.fps = 0.0
        
        # Setup QoS for camera data (compatible with Go2 camera)
        camera_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Setup subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            camera_qos
        )
        
        self.people_sub = self.create_subscription(
            String,
            '/human_detection/people',
            self.people_callback,
            10
        )
        
        self.gestures_sub = self.create_subscription(
            String,
            '/human_detection/gestures',
            self.gestures_callback,
            10
        )
        
        self.state_sub = self.create_subscription(
            String,
            '/interaction/state',
            self.state_callback,
            10
        )
        
        # Subscribe to combined gestures for multi-human display
        self.combined_gestures_sub = self.create_subscription(
            String,
            '/human_detection/combined_gestures',
            self.combined_gestures_callback,
            10
        )
        
        # Subscribe to interaction events
        self.events_sub = self.create_subscription(
            String,
            '/interaction/events',
            self.events_callback,
            10
        )
        
        # Setup display timer
        self.display_timer = self.create_timer(
            0.033,  # ~30 FPS
            self.update_display
        )
        
        self.get_logger().info('Simple Camera Display Node initialized')
        self.get_logger().info('Press "q" to quit, "s" to save screenshot')
    
    def image_callback(self, msg: Image):
        """Handle incoming camera images"""
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.current_image = cv_image
            
            # Update FPS
            self.frame_count += 1
            current_time = time.time()
            if current_time - self.last_fps_time >= 1.0:
                self.fps = self.frame_count / (current_time - self.last_fps_time)
                self.frame_count = 0
                self.last_fps_time = current_time
                
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')
    
    def people_callback(self, msg: String):
        """Handle human detection results"""
        try:
            data = json.loads(msg.data)
            detection = data.get('detection', {})
            human_id = detection.get('id', 0)
            
            # Store detection data by human ID
            self.current_detections[human_id] = data
            
            # Clean up old detections (older than 5 seconds)
            current_time = time.time()
            expired_humans = []
            for hid, det_data in self.current_detections.items():
                if current_time - det_data.get('timestamp', current_time) > 5.0:
                    expired_humans.append(hid)
            
            for hid in expired_humans:
                del self.current_detections[hid]
                if hid in self.current_gestures:
                    del self.current_gestures[hid]
                    
        except Exception as e:
            self.get_logger().error(f'Error processing people data: {e}')
    
    def gestures_callback(self, msg: String):
        """Handle gesture recognition results"""
        try:
            data = json.loads(msg.data)
            human_id = data.get('human_id', 0)
            
            # Store gesture data by human ID
            self.current_gestures[human_id] = data
            
        except Exception as e:
            self.get_logger().error(f'Error processing gestures: {e}')
    
    def state_callback(self, msg: String):
        """Handle interaction state updates"""
        try:
            data = json.loads(msg.data)
            self.current_state = data.get('current_state', 'Unknown')
        except Exception as e:
            self.get_logger().error(f'Error processing state data: {e}')
    
    def combined_gestures_callback(self, msg: String):
        """Handle combined multi-human gesture data"""
        try:
            data = json.loads(msg.data)
            self.combined_gestures = data
        except Exception as e:
            self.get_logger().error(f'Error processing combined gestures: {e}')
    
    def events_callback(self, msg: String):
        """Handle interaction events"""
        try:
            data = json.loads(msg.data)
            # Keep only recent events (last 10)
            self.interaction_events.append(data)
            if len(self.interaction_events) > 10:
                self.interaction_events.pop(0)
        except Exception as e:
            self.get_logger().error(f'Error processing events: {e}')
    
    def update_display(self):
        """Update the camera display with overlays"""
        try:
            if self.current_image is None:
                # Create a black placeholder image when no camera feed
                display_image = self.create_waiting_screen()
            else:
                # Create a copy for display
                display_image = self.current_image.copy()
                
                # Draw YOLO detections
                self.draw_yolo_detections(display_image)
                
                # Draw MediaPipe gestures
                self.draw_gesture_info(display_image)
                
                # Draw interaction state
                self.draw_state_info(display_image)
                
                # Draw performance info
                self.draw_performance_info(display_image)
            
            # Always display the image (either camera feed or waiting screen)
            cv2.imshow('Robot Dog Petting Zoo - Camera Feed', display_image)
            
            # Handle key presses
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.get_logger().info('Quit requested by user')
                rclpy.shutdown()
            elif key == ord('s'):
                self.save_screenshot(display_image)
                
        except Exception as e:
            self.get_logger().error(f'Error updating display: {e}')
    
    def create_waiting_screen(self) -> np.ndarray:
        """Create a waiting screen when no camera feed is available"""
        try:
            # Create a black image (640x480 default size)
            height, width = 480, 640
            waiting_image = np.zeros((height, width, 3), dtype=np.uint8)
            
            # Main waiting message
            main_text = "Waiting for Robot Dog Camera Feed..."
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.8
            color = (255, 255, 255)  # White text
            thickness = 2
            
            # Get text size and center it
            text_size = cv2.getTextSize(main_text, font, font_scale, thickness)[0]
            text_x = (width - text_size[0]) // 2
            text_y = (height - text_size[1]) // 2
            
            # Draw main message
            cv2.putText(waiting_image, main_text, (text_x, text_y), 
                       font, font_scale, color, thickness)
            
            # Instructions
            instructions = [
                "Press 'q' to quit",
                "Waiting for /camera/image_raw topic...",
                f"Current State: {self.current_state}"
            ]
            
            # Draw instructions
            for i, instruction in enumerate(instructions):
                inst_size = cv2.getTextSize(instruction, font, 0.5, 1)[0]
                inst_x = (width - inst_size[0]) // 2
                inst_y = text_y + 60 + (i * 30)
                cv2.putText(waiting_image, instruction, (inst_x, inst_y), 
                           font, 0.5, (200, 200, 200), 1)
            
            # Add a subtle border
            cv2.rectangle(waiting_image, (10, 10), (width-10, height-10), 
                         (100, 100, 100), 2)
            
            # Add timestamp
            timestamp_text = f"Waiting since: {time.strftime('%H:%M:%S')}"
            cv2.putText(waiting_image, timestamp_text, (20, height-20), 
                       font, 0.4, (150, 150, 150), 1)
            
            return waiting_image
            
        except Exception as e:
            self.get_logger().error(f'Error creating waiting screen: {e}')
            # Return a simple black image as fallback
            return np.zeros((480, 640, 3), dtype=np.uint8)
    
    def draw_yolo_detections(self, image: np.ndarray):
        """Draw YOLO human detection bounding boxes with IDs"""
        try:
            for human_id, detection_data in self.current_detections.items():
                detection = detection_data.get('detection', {})
                bbox = detection.get('bbox', [])
                confidence = detection.get('confidence', 0.0)
                distance = detection_data.get('distance', 0.0)
                zone = detection_data.get('zone', 'unknown')
                
                if len(bbox) == 4:
                    x1, y1, x2, y2 = bbox
                    
                    # Draw bounding box
                    color = (0, 255, 0)  # Green for humans
                    cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
                    
                    # Draw label with human ID
                    label = f"Human {human_id}: {confidence:.2f}"
                    label_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)[0]
                    cv2.rectangle(image, (x1, y1 - label_size[1] - 10), 
                                (x1 + label_size[0], y1), color, -1)
                    cv2.putText(image, label, (x1, y1 - 5), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                    
                    # Draw distance and zone info
                    info_text = f"Dist: {distance:.1f}m | Zone: {zone}"
                    cv2.putText(image, info_text, (x1, y2 + 20), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
                    
                    # Draw gesture info for this human
                    if human_id in self.current_gestures:
                        gesture_data = self.current_gestures[human_id]
                        gestures = gesture_data.get('gestures', [])
                        gesture_states = gesture_data.get('gesture_states', {})
                        
                        if gestures:
                            gesture_y = y2 + 45
                            for gesture in gestures[:3]:  # Show max 3 gestures per human
                                state = gesture_states.get(gesture, 'unknown')
                                state_color = {
                                    'new': (0, 255, 255),      # Cyan for NEW
                                    'ongoing': (0, 165, 255),  # Orange for ONGOING
                                    'ended': (128, 128, 128)   # Gray for ENDED
                                }.get(state, (255, 255, 255))
                                
                                gesture_text = f"{gesture} ({state.upper()})"
                                cv2.putText(image, gesture_text, (x1, gesture_y), 
                                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, state_color, 1)
                                gesture_y += 15
                    
        except Exception as e:
            self.get_logger().error(f'Error drawing YOLO detections: {e}')
    
    def draw_gesture_info(self, image: np.ndarray):
        """Draw priority-based gesture information and interaction queue"""
        try:
            y_offset = 30
            
            # Draw combined gesture priority info
            if self.combined_gestures:
                prioritized = self.combined_gestures.get('prioritized_gestures', [])
                total_humans = self.combined_gestures.get('total_humans', 0)
                
                # Header
                header_text = f"Multi-Human Gestures ({total_humans} humans):"
                cv2.putText(image, header_text, (10, y_offset), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                y_offset += 25
                
                # Show prioritized gestures
                for i, gesture_data in enumerate(prioritized[:5]):  # Show top 5
                    human_id = gesture_data.get('human_id', 0)
                    gestures = gesture_data.get('gestures', [])
                    priority = gesture_data.get('priority_score', 0)
                    
                    priority_text = f"#{i+1} Human {human_id}: {', '.join(gestures)} (Priority: {priority:.0f})"
                    color = (0, 255, 255) if i == 0 else (200, 200, 200)  # Highlight top priority
                    cv2.putText(image, priority_text, (20, y_offset), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
                    y_offset += 20
            
            # Draw recent interaction events
            if self.interaction_events:
                y_offset += 10
                events_text = "Recent Events:"
                cv2.putText(image, events_text, (10, y_offset), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 100, 255), 2)
                y_offset += 25
                
                for event in self.interaction_events[-3:]:  # Show last 3 events
                    event_type = event.get('event', 'unknown')
                    human_id = event.get('human_id', 'N/A')
                    event_text = f"• {event_type} (Human {human_id})"
                    cv2.putText(image, event_text, (20, y_offset), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 150, 255), 1)
                    y_offset += 18
                    
        except Exception as e:
            self.get_logger().error(f'Error drawing gesture info: {e}')
    
    def draw_state_info(self, image: np.ndarray):
        """Draw interaction state information"""
        try:
            # Draw state in top-right corner
            state_text = f"State: {self.current_state}"
            text_size = cv2.getTextSize(state_text, cv2.FONT_HERSHEY_SIMPLEX, 0.8, 2)[0]
            x = image.shape[1] - text_size[0] - 20
            y = 30
            
            # Background rectangle
            cv2.rectangle(image, (x - 10, y - text_size[1] - 10), 
                         (x + text_size[0] + 10, y + 10), (0, 0, 0), -1)
            
            # State text
            color = (0, 255, 0) if self.current_state == "patrol" else (255, 255, 0)
            cv2.putText(image, state_text, (x, y), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
            
        except Exception as e:
            self.get_logger().error(f'Error drawing state info: {e}')
    
    def draw_performance_info(self, image: np.ndarray):
        """Draw performance metrics"""
        try:
            # FPS counter
            fps_text = f"FPS: {self.fps:.1f}"
            cv2.putText(image, fps_text, (10, image.shape[0] - 20), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            # Detection count
            detection_count = len(self.current_detections)
            detection_text = f"Humans: {detection_count}"
            cv2.putText(image, detection_text, (10, image.shape[0] - 40), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            # Gesture count
            gesture_count = len(self.current_gestures)
            gesture_text = f"Gestures: {gesture_count}"
            cv2.putText(image, gesture_text, (10, image.shape[0] - 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
        except Exception as e:
            self.get_logger().error(f'Error drawing performance info: {e}')
    
    def save_screenshot(self, image: np.ndarray):
        """Save a screenshot of the current display"""
        try:
            timestamp = int(time.time())
            filename = f"camera_screenshot_{timestamp}.png"
            cv2.imwrite(filename, image)
            self.get_logger().info(f'Screenshot saved as {filename}')
        except Exception as e:
            self.get_logger().error(f'Error saving screenshot: {e}')
    
    def on_shutdown(self):
        """Clean up on shutdown"""
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    
    node = SimpleCameraDisplay()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        try:
            node.on_shutdown()
            node.destroy_node()
        except:
            pass
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main() 