import rclpy
import pygame
from geometry_msgs.msg import Twist
import time
import threading

class XboxTeleopNode:
    def __init__(self):
        self.node = rclpy.create_node('xbox_teleop')
        self.publisher = self.node.create_publisher(Twist, '/cmd_vel_joy', 10)
        
        self.cmd_vel_msg = Twist()
        
        # Controller settings
        self.max_linear_speed = 0.5
        self.max_angular_speed = 0.5
        self.deadzone = 0.1  # Ignore small stick movements
        
        # Initialize pygame
        pygame.init()
        pygame.joystick.init()
        
        # Check for controller
        if pygame.joystick.get_count() == 0:
            print("No controller detected!")
            return
        
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        print(f"Controller detected: {self.joystick.get_name()}")
        print("Xbox Controller Teleop Node initiated")
        print("Left stick: Move forward/backward and turn")
        print("Press 'B' button (button 1) to exit")

    def apply_deadzone(self, value, deadzone):
        """Apply deadzone to joystick input"""
        if abs(value) < deadzone:
            return 0.0
        return value

    def start(self):
        """Start the controller input loop"""
        if not hasattr(self, 'joystick'):
            return
            
        clock = pygame.time.Clock()
        running = True
        
        try:
            while running and rclpy.ok():
                pygame.event.pump()
                
                # Get joystick axes
                # Left stick Y-axis (inverted for forward/backward)
                left_stick_y = -self.joystick.get_axis(1)  # Axis 1, inverted
                # Left stick X-axis (for turning)
                left_stick_x = self.joystick.get_axis(0)   # Axis 0
                
                # Apply deadzone
                left_stick_y = self.apply_deadzone(left_stick_y, self.deadzone)
                left_stick_x = self.apply_deadzone(left_stick_x, self.deadzone)
                
                # Set velocities
                self.cmd_vel_msg.linear.x = left_stick_y * self.max_linear_speed
                self.cmd_vel_msg.angular.z = -left_stick_x * self.max_angular_speed  # Inverted for intuitive turning
                
                # Publish command
                self.publish_cmd_vel()
                
                # Check for exit button (B button)
                if self.joystick.get_button(1):  # B button
                    print("B button pressed - exiting")
                    running = False
                
                # Optional: Print current values (uncomment for debugging)
                # if abs(left_stick_y) > 0 or abs(left_stick_x) > 0:
                #     print(f"Linear: {self.cmd_vel_msg.linear.x:.2f}, Angular: {self.cmd_vel_msg.angular.z:.2f}")
                
                clock.tick(60)  # 60 Hz update rate
                
        except KeyboardInterrupt:
            print("Keyboard interrupt received")
        finally:
            # Stop the robot
            self.cmd_vel_msg.linear.x = 0.0
            self.cmd_vel_msg.angular.z = 0.0
            self.publish_cmd_vel()
            pygame.quit()

    def publish_cmd_vel(self):
        """Publish the velocity command"""
        self.publisher.publish(self.cmd_vel_msg)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        xbox_teleop_node = XboxTeleopNode()
        
        # Start ROS2 spinning in a separate thread
        spin_thread = threading.Thread(target=rclpy.spin, args=(xbox_teleop_node.node,))
        spin_thread.daemon = True
        spin_thread.start()
        
        # Start controller input loop in main thread
        xbox_teleop_node.start()
        
    except KeyboardInterrupt:
        print("Keyboard interrupt in main")
    finally:
        if 'xbox_teleop_node' in locals():
            xbox_teleop_node.node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
