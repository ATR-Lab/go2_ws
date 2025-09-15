import rclpy
import sys
import select
import termios
import tty
from geometry_msgs.msg import Twist
import threading
import time

class Go2KeyboardTeleopNode:
    def __init__(self):
        self.node = rclpy.create_node('go2_keyboard_teleop')
        self.publisher = self.node.create_publisher(Twist, '/cmd_vel_joy', 10)
        
        self.cmd_vel_msg = Twist()
        
        # Movement settings
        self.max_linear_speed = 0.5
        self.max_angular_speed = 0.5
        self.speed_increment = 0.1
        
        # Current speeds
        self.current_linear = 0.0
        self.current_angular = 0.0
        
        # Terminal settings for non-blocking input
        self.old_settings = None
        try:
            self.old_settings = termios.tcgetattr(sys.stdin)
        except termios.error:
            print("ERROR: This node requires a terminal (TTY) for keyboard input.")
            print("Please run directly in a terminal instead of through launch:")
            print("  ros2 run go2_teleop teleop_keyboard")
            raise RuntimeError("No TTY available for keyboard input")
        
        print("Go2 Keyboard Teleop Node initiated")
        print("Controls:")
        print("  W/↑    : Move forward")
        print("  S/↓    : Move backward") 
        print("  A/←    : Turn left")
        print("  D/→    : Turn right")
        print("  Q      : Increase linear speed")
        print("  Z      : Decrease linear speed")
        print("  E      : Increase angular speed")
        print("  C      : Decrease angular speed")
        print("  SPACE  : Stop")
        print("  ESC/q  : Quit")
        print(f"Current speeds - Linear: {self.max_linear_speed:.1f}, Angular: {self.max_angular_speed:.1f}")
        print("Press keys to control the robot...")

    def get_key(self):
        """Get a single keypress without requiring Enter"""
        try:
            tty.setraw(sys.stdin.fileno())
            if select.select([sys.stdin], [], [], 0.1)[0]:
                key = sys.stdin.read(1)
                return key
            return None
        finally:
            if self.old_settings is not None:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)

    def process_key(self, key):
        """Process keyboard input and update velocities"""
        if key is None:
            return True
            
        # Convert to lowercase for easier handling
        key_lower = key.lower()
        
        # Movement keys
        if key_lower == 'w' or key == '\x1b[A':  # W or Up arrow
            self.current_linear = self.max_linear_speed
            self.current_angular = 0.0
        elif key_lower == 's' or key == '\x1b[B':  # S or Down arrow
            self.current_linear = -self.max_linear_speed
            self.current_angular = 0.0
        elif key_lower == 'a' or key == '\x1b[D':  # A or Left arrow
            self.current_linear = 0.0
            self.current_angular = self.max_angular_speed
        elif key_lower == 'd' or key == '\x1b[C':  # D or Right arrow
            self.current_linear = 0.0
            self.current_angular = -self.max_angular_speed
        
        # Speed adjustment keys
        elif key_lower == 'q':
            self.max_linear_speed = min(1.0, self.max_linear_speed + self.speed_increment)
            print(f"Linear speed: {self.max_linear_speed:.1f}")
        elif key_lower == 'z':
            self.max_linear_speed = max(0.1, self.max_linear_speed - self.speed_increment)
            print(f"Linear speed: {self.max_linear_speed:.1f}")
        elif key_lower == 'e':
            self.max_angular_speed = min(1.0, self.max_angular_speed + self.speed_increment)
            print(f"Angular speed: {self.max_angular_speed:.1f}")
        elif key_lower == 'c':
            self.max_angular_speed = max(0.1, self.max_angular_speed - self.speed_increment)
            print(f"Angular speed: {self.max_angular_speed:.1f}")
        
        # Stop key
        elif key == ' ':  # Space
            self.current_linear = 0.0
            self.current_angular = 0.0
            print("Stopped")
        
        # Quit keys
        elif key == '\x1b' or key_lower == 'q':  # ESC or q
            print("Quitting...")
            return False
        
        # Handle arrow key sequences (they come as escape sequences)
        elif key == '\x1b':
            # This is the start of an escape sequence, need to read more
            next_chars = sys.stdin.read(2)
            if next_chars == '[A':  # Up arrow
                self.current_linear = self.max_linear_speed
                self.current_angular = 0.0
            elif next_chars == '[B':  # Down arrow
                self.current_linear = -self.max_linear_speed
                self.current_angular = 0.0
            elif next_chars == '[D':  # Left arrow
                self.current_linear = 0.0
                self.current_angular = self.max_angular_speed
            elif next_chars == '[C':  # Right arrow
                self.current_linear = 0.0
                self.current_angular = -self.max_angular_speed
        
        return True

    def start(self):
        """Start the keyboard input loop"""
        try:
            running = True
            while running and rclpy.ok():
                # Get keyboard input
                key = self.get_key()
                
                # Process the key
                running = self.process_key(key)
                
                # Update and publish velocity
                self.cmd_vel_msg.linear.x = self.current_linear
                self.cmd_vel_msg.angular.z = self.current_angular
                self.publish_cmd_vel()
                
                # Decay velocities (stop if no key pressed)
                if key is None:
                    self.current_linear *= 0.9
                    self.current_angular *= 0.9
                    
                    # Stop completely if very small
                    if abs(self.current_linear) < 0.01:
                        self.current_linear = 0.0
                    if abs(self.current_angular) < 0.01:
                        self.current_angular = 0.0
                
                time.sleep(0.05)  # 20 Hz update rate
                
        except KeyboardInterrupt:
            print("\nKeyboard interrupt received")
        finally:
            # Stop the robot
            self.cmd_vel_msg.linear.x = 0.0
            self.cmd_vel_msg.angular.z = 0.0
            self.publish_cmd_vel()
            # Restore terminal settings
            if self.old_settings is not None:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)

    def publish_cmd_vel(self):
        """Publish the velocity command"""
        self.publisher.publish(self.cmd_vel_msg)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        keyboard_teleop_node = Go2KeyboardTeleopNode()
        
        # Start ROS2 spinning in a separate thread
        spin_thread = threading.Thread(target=rclpy.spin, args=(keyboard_teleop_node.node,))
        spin_thread.daemon = True
        spin_thread.start()
        
        # Start keyboard input loop in main thread
        keyboard_teleop_node.start()
        
    except KeyboardInterrupt:
        print("Keyboard interrupt in main")
    finally:
        if 'keyboard_teleop_node' in locals():
            keyboard_teleop_node.node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
