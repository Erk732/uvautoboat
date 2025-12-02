#!/usr/bin/env python3
"""
Keyboard Teleop for WAM-V Boat - War Thunder / GTA5 Naval Style

Controls like naval games: throttle persists, rudder for steering.

Controls:
    W/↑ : Increase throttle (speed up)
    S/↓ : Decrease throttle (slow down / reverse)
    A/← : Rudder Left (turn while maintaining throttle)
    D/→ : Rudder Right (turn while maintaining throttle)
    Q   : Quick turn left (sharp)
    E   : Quick turn right (sharp)
    Space : All stop (zero throttle, center rudder)
    X   : Emergency reverse (full reverse)
    +/= : Increase max thrust power
    -   : Decrease max thrust power
    R   : Center rudder (straighten out)
    H   : Show help
    ESC/Ctrl+C : Quit

Throttle: Persists between keypresses (like real boat throttle lever)
Rudder: Returns to center when A/D released (smooth steering)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import sys
import termios
import tty
import select
import time


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        
        # Publishers for left and right thrusters
        self.left_pub = self.create_publisher(Float64, '/wamv/thrusters/left/thrust', 10)
        self.right_pub = self.create_publisher(Float64, '/wamv/thrusters/right/thrust', 10)
        
        # ===========================================
        # WAR THUNDER / GTA5 STYLE PARAMETERS
        # (Tuned for smooth, realistic boat handling)
        # ===========================================
        
        # Throttle (persists - like a real throttle lever)
        self.throttle = 0.0          # Current throttle: -1.0 (full reverse) to 1.0 (full ahead)
        self.throttle_step = 0.05    # Smaller steps = smoother acceleration
        self.target_throttle = 0.0   # Target throttle for smooth ramping
        self.throttle_ramp_rate = 0.03  # How fast throttle ramps to target (per update)
        
        # Rudder (returns to center - for steering)
        self.rudder = 0.0            # Current rudder: -1.0 (full left) to 1.0 (full right)
        self.rudder_step = 0.12      # How much rudder changes per keypress
        self.rudder_decay = 0.02     # Slower decay = smoother steering
        self.rudder_held = False     # Track if rudder key is being held
        
        # Thrust power (scales the output)
        self.max_thrust = 800.0      # Maximum thrust in Newtons
        self.thrust_step = 100.0     # Increment for +/- keys
        
        # Timing
        self.last_rudder_key_time = 0.0
        self.rudder_key_timeout = 0.15  # Rudder starts returning to center after this
        self.last_throttle_key_time = 0.0
        
        # Store original terminal settings
        self.old_settings = termios.tcgetattr(sys.stdin)
        
        # Timer for continuous thrust updates - HIGHER RATE for smoothness
        self.create_timer(0.033, self.update_thrust)  # 30 Hz update (smoother)
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('🎮 WAR THUNDER / GTA5 NAVAL TELEOP STARTED')
        self.get_logger().info('=' * 60)
        self.print_instructions()
        
    def print_instructions(self):
        print("\n" + "=" * 60)
        print("🚤 WAM-V Naval Teleop - War Thunder / GTA5 Style")
        print("=" * 60)
        print("THROTTLE (persists like a lever):")
        print("  W/↑    : Increase throttle (faster)")
        print("  S/↓    : Decrease throttle (slower/reverse)")
        print("  SPACE  : All stop (zero throttle)")
        print("  X      : Emergency full reverse")
        print("")
        print("RUDDER (returns to center):")
        print("  A/←    : Steer left")
        print("  D/→    : Steer right")
        print("  Q      : Hard left turn")
        print("  E      : Hard right turn")
        print("  R      : Center rudder")
        print("")
        print("POWER:")
        print("  +/=    : Increase max thrust")
        print("  -      : Decrease max thrust")
        print("")
        print("  H      : Show this help")
        print("  Ctrl+C : Quit")
        print("=" * 60)
        print(f"Max thrust: {self.max_thrust:.0f} N")
        print("=" * 60 + "\n")
        
    def get_key_nonblocking(self):
        """Non-blocking key read with timeout."""
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.02)
        key = ''
        if rlist:
            key = sys.stdin.read(1)
            # Handle arrow keys (escape sequences)
            if key == '\x1b':
                key += sys.stdin.read(2)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        return key
    
    def update_thrust(self):
        """
        Called at 30 Hz to update thrust based on throttle + rudder.
        This creates the smooth War Thunder / GTA5 feel.
        
        Key improvements for smoothness:
        1. Throttle ramping - smooth acceleration/deceleration
        2. Slower rudder decay - less jerky steering
        3. Higher update rate - smoother movement
        """
        current_time = time.time()
        
        # =========================================
        # THROTTLE RAMPING (smooth acceleration)
        # =========================================
        # Smoothly ramp actual throttle toward target throttle
        if abs(self.throttle - self.target_throttle) > 0.01:
            if self.throttle < self.target_throttle:
                self.throttle = min(self.target_throttle, 
                                   self.throttle + self.throttle_ramp_rate)
            else:
                self.throttle = max(self.target_throttle, 
                                   self.throttle - self.throttle_ramp_rate)
        else:
            self.throttle = self.target_throttle
        
        # =========================================
        # RUDDER DECAY (smooth return to center)
        # =========================================
        if current_time - self.last_rudder_key_time > self.rudder_key_timeout:
            if abs(self.rudder) > 0.01:
                if self.rudder > 0:
                    self.rudder = max(0.0, self.rudder - self.rudder_decay)
                else:
                    self.rudder = min(0.0, self.rudder + self.rudder_decay)
        
        # =========================================
        # DIFFERENTIAL THRUST CALCULATION
        # =========================================
        # Base thrust from throttle
        base_thrust = self.throttle * self.max_thrust
        
        # Rudder effect (more pronounced at higher speeds, like real boats)
        speed_factor = 0.3 + 0.7 * abs(self.throttle)  # Rudder more effective at speed
        rudder_effect = self.rudder * self.max_thrust * 0.5 * speed_factor
        
        # Apply to left/right thrusters
        # Positive rudder = turn right = more left thrust, less right thrust
        left_thrust = base_thrust + rudder_effect
        right_thrust = base_thrust - rudder_effect
        
        # Clamp to max thrust
        left_thrust = max(-self.max_thrust, min(self.max_thrust, left_thrust))
        right_thrust = max(-self.max_thrust, min(self.max_thrust, right_thrust))
        
        # Publish
        left_msg = Float64()
        left_msg.data = left_thrust
        right_msg = Float64()
        right_msg.data = right_thrust
        
        self.left_pub.publish(left_msg)
        self.right_pub.publish(right_msg)
        
        # Visual HUD
        throttle_bar = self.make_bar(self.throttle, 10)
        rudder_bar = self.make_rudder_bar(self.rudder, 10)
        
        # Direction indicator
        if abs(self.throttle) < 0.05:
            direction = "⏹ STOP"
        elif self.throttle > 0:
            direction = "⬆ AHEAD"
        else:
            direction = "⬇ ASTERN"
        
        # Show target throttle when ramping
        target_indicator = ""
        if abs(self.target_throttle - self.throttle) > 0.02:
            target_indicator = f"→{self.target_throttle*100:+.0f}%"
        
        print(f"\r{direction:10} | Throttle [{throttle_bar}] {self.throttle*100:+4.0f}%{target_indicator:>6} | "
              f"Rudder [{rudder_bar}] {self.rudder*100:+4.0f}% | "
              f"L:{left_thrust:+5.0f} R:{right_thrust:+5.0f}   ", end='', flush=True)
    
    def make_bar(self, value, width):
        """Make a visual bar for throttle (-1 to 1)."""
        half = width // 2
        filled = int(abs(value) * half)
        if value >= 0:
            return '=' * half + '|' + '█' * filled + '░' * (half - filled)
        else:
            return '░' * (half - filled) + '█' * filled + '|' + '=' * half
    
    def make_rudder_bar(self, value, width):
        """Make a visual bar for rudder (-1 to 1)."""
        half = width // 2
        pos = int(value * half)
        bar = ['─'] * (width + 1)
        bar[half] = '│'
        bar[half + pos] = '◆'
        return ''.join(bar)
    
    def run(self):
        """Main teleop loop."""
        try:
            while rclpy.ok():
                key = self.get_key_nonblocking()
                
                if key == '':
                    rclpy.spin_once(self, timeout_sec=0.01)
                    continue
                    
                # Check for Ctrl+C
                if key == '\x03':
                    break
                
                # ===========================================
                # THROTTLE CONTROLS (persists with smooth ramping)
                # ===========================================
                if key.lower() == 'w' or key == '\x1b[A':  # W or Up arrow
                    self.target_throttle = min(1.0, self.target_throttle + self.throttle_step)
                    self.last_throttle_key_time = time.time()
                    
                elif key.lower() == 's' or key == '\x1b[B':  # S or Down arrow
                    self.target_throttle = max(-1.0, self.target_throttle - self.throttle_step)
                    self.last_throttle_key_time = time.time()
                    
                elif key == ' ':  # Space - All stop
                    self.target_throttle = 0.0
                    self.throttle = 0.0
                    self.rudder = 0.0
                    print("\n⚓ ALL STOP")
                    
                elif key.lower() == 'x':  # Emergency reverse
                    self.target_throttle = -1.0
                    self.throttle = -1.0  # Immediate for emergency
                    print("\n🔴 EMERGENCY REVERSE")
                
                # ===========================================
                # RUDDER CONTROLS (returns to center)
                # ===========================================
                elif key.lower() == 'a' or key == '\x1b[D':  # A or Left arrow
                    self.rudder = max(-1.0, self.rudder - self.rudder_step)
                    self.last_rudder_key_time = time.time()
                    
                elif key.lower() == 'd' or key == '\x1b[C':  # D or Right arrow
                    self.rudder = min(1.0, self.rudder + self.rudder_step)
                    self.last_rudder_key_time = time.time()
                    
                elif key.lower() == 'q':  # Hard left
                    self.rudder = -1.0
                    self.last_rudder_key_time = time.time()
                    
                elif key.lower() == 'e':  # Hard right
                    self.rudder = 1.0
                    self.last_rudder_key_time = time.time()
                    
                elif key.lower() == 'r':  # Center rudder
                    self.rudder = 0.0
                
                # ===========================================
                # POWER CONTROLS
                # ===========================================
                elif key in ['+', '=']:  # Increase max thrust
                    self.max_thrust = min(1000.0, self.max_thrust + self.thrust_step)
                    print(f"\n🔼 Max thrust: {self.max_thrust:.0f} N")
                    
                elif key == '-':  # Decrease max thrust
                    self.max_thrust = max(100.0, self.max_thrust - self.thrust_step)
                    print(f"\n🔽 Max thrust: {self.max_thrust:.0f} N")
                    
                elif key.lower() == 'h':  # Help
                    self.print_instructions()
                
                rclpy.spin_once(self, timeout_sec=0.01)
                    
        except Exception as e:
            self.get_logger().error(f'Error: {e}')
        finally:
            # Stop thrusters on exit
            self.throttle = 0.0
            self.target_throttle = 0.0
            self.rudder = 0.0
            left_msg = Float64()
            left_msg.data = 0.0
            right_msg = Float64()
            right_msg.data = 0.0
            self.left_pub.publish(left_msg)
            self.right_pub.publish(right_msg)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
            print("\n\n⚓ Teleop stopped. All stop.")


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
