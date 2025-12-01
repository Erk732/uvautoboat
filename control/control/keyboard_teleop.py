#!/usr/bin/env python3
"""
Simple keyboard teleop for WAMv boat.
Directly controls left/right thrusters.

Controls:
    W/↑ : Forward (both thrusters positive)
    S/↓ : Backward (both thrusters negative)
    A/← : Turn Left (right thruster forward, left backward)
    D/→ : Turn Right (left thruster forward, right backward)
    Q   : Forward-Left
    E   : Forward-Right
    Space : Stop (zero both thrusters)
    +/= : Increase thrust power
    -   : Decrease thrust power
    ESC/Ctrl+C : Quit
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import sys
import termios
import tty
import select

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        
        # Publishers for left and right thrusters
        self.left_pub = self.create_publisher(Float64, '/wamv/thrusters/left/thrust', 10)
        self.right_pub = self.create_publisher(Float64, '/wamv/thrusters/right/thrust', 10)
        
        # Thrust settings
        self.thrust_power = 500.0  # Current thrust power (0-1000)
        self.thrust_step = 100.0   # Increment step
        self.max_thrust = 1000.0
        self.min_thrust = 100.0
        
        # Current thrust values
        self.left_thrust = 0.0
        self.right_thrust = 0.0
        
        # Store original terminal settings
        self.old_settings = termios.tcgetattr(sys.stdin)
        
        self.get_logger().info('=' * 50)
        self.get_logger().info('🎮 KEYBOARD TELEOP STARTED')
        self.get_logger().info('=' * 50)
        self.print_instructions()
        
    def print_instructions(self):
        print("\n" + "=" * 50)
        print("🚤 WAMv Keyboard Teleop")
        print("=" * 50)
        print("Controls:")
        print("  W/↑    : Forward")
        print("  S/↓    : Backward")
        print("  A/←    : Turn Left")
        print("  D/→    : Turn Right")
        print("  Q      : Forward-Left")
        print("  E      : Forward-Right")
        print("  SPACE  : Stop")
        print("  +/=    : Increase power")
        print("  -      : Decrease power")
        print("  Ctrl+C : Quit")
        print("=" * 50)
        print(f"Current thrust power: {self.thrust_power:.0f}")
        print("=" * 50 + "\n")
        
    def get_key(self):
        """Get a single keypress."""
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        return key
    
    def get_key_nonblocking(self):
        """Non-blocking key read with timeout."""
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
            # Handle arrow keys (escape sequences)
            if key == '\x1b':
                key += sys.stdin.read(2)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        return key
    
    def publish_thrust(self, left, right):
        """Publish thrust values to both motors."""
        self.left_thrust = left
        self.right_thrust = right
        
        left_msg = Float64()
        left_msg.data = left
        right_msg = Float64()
        right_msg.data = right
        
        self.left_pub.publish(left_msg)
        self.right_pub.publish(right_msg)
        
        # Visual feedback
        direction = "⏹ STOP" if left == 0 and right == 0 else ""
        if left > 0 and right > 0:
            direction = "⬆ FORWARD"
        elif left < 0 and right < 0:
            direction = "⬇ BACKWARD"
        elif left < right:
            direction = "⬅ LEFT"
        elif left > right:
            direction = "➡ RIGHT"
        
        print(f"\r{direction:15} | L: {left:+7.1f} | R: {right:+7.1f} | Power: {self.thrust_power:.0f}   ", end='', flush=True)
    
    def run(self):
        """Main teleop loop."""
        try:
            while rclpy.ok():
                key = self.get_key_nonblocking()
                
                if key == '':
                    continue
                    
                # Check for Ctrl+C
                if key == '\x03':
                    break
                
                # Movement controls
                if key.lower() == 'w' or key == '\x1b[A':  # W or Up arrow
                    self.publish_thrust(self.thrust_power, self.thrust_power)
                    
                elif key.lower() == 's' or key == '\x1b[B':  # S or Down arrow
                    self.publish_thrust(-self.thrust_power, -self.thrust_power)
                    
                elif key.lower() == 'a' or key == '\x1b[D':  # A or Left arrow
                    self.publish_thrust(-self.thrust_power * 0.5, self.thrust_power * 0.5)
                    
                elif key.lower() == 'd' or key == '\x1b[C':  # D or Right arrow
                    self.publish_thrust(self.thrust_power * 0.5, -self.thrust_power * 0.5)
                    
                elif key.lower() == 'q':  # Forward-Left
                    self.publish_thrust(self.thrust_power * 0.3, self.thrust_power)
                    
                elif key.lower() == 'e':  # Forward-Right
                    self.publish_thrust(self.thrust_power, self.thrust_power * 0.3)
                    
                elif key == ' ':  # Space - Stop
                    self.publish_thrust(0.0, 0.0)
                    
                elif key in ['+', '=']:  # Increase thrust
                    self.thrust_power = min(self.thrust_power + self.thrust_step, self.max_thrust)
                    print(f"\n🔼 Thrust power: {self.thrust_power:.0f}")
                    
                elif key == '-':  # Decrease thrust
                    self.thrust_power = max(self.thrust_power - self.thrust_step, self.min_thrust)
                    print(f"\n🔽 Thrust power: {self.thrust_power:.0f}")
                    
                elif key.lower() == 'h':  # Help
                    self.print_instructions()
                    
        except Exception as e:
            self.get_logger().error(f'Error: {e}')
        finally:
            # Stop thrusters on exit
            self.publish_thrust(0.0, 0.0)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
            print("\n\n🛑 Teleop stopped. Thrusters zeroed.")


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_thrust(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
