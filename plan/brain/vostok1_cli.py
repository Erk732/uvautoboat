#!/usr/bin/env python3
"""
Mission CLI - Terminal-based mission control for Vostok1 / Sputnik
Mission CLI - Contrôle de mission via terminal

Use this when the web dashboard is unavailable.

MODES:
    --mode vostok1   (default) Use integrated Vostok1 navigation
    --mode modular   Use modular Sputnik planner + separate controller

Usage (Integrated - Vostok1):
    # Generate waypoints with default parameters
    ros2 run plan vostok1_cli generate
    
    # Generate waypoints with custom parameters
    ros2 run plan vostok1_cli generate --lanes 8 --length 50 --width 20
    
    # Start mission
    ros2 run plan vostok1_cli start
    
    # Stop mission
    ros2 run plan vostok1_cli stop
    
    # Resume mission
    ros2 run plan vostok1_cli resume
    
    # Reset mission (clear waypoints)
    ros2 run plan vostok1_cli reset
    
    # Set PID parameters
    ros2 run plan vostok1_cli pid --kp 400 --ki 20 --kd 100
    
    # Set speed
    ros2 run plan vostok1_cli speed --base 500 --max 800
    
    # Show current status
    ros2 run plan vostok1_cli status
    
    # Interactive mode
    ros2 run plan vostok1_cli interactive

Usage (Modular - Sputnik Planner):
    # Generate waypoints
    ros2 run plan vostok1_cli --mode modular generate --lanes 8 --length 15 --width 5
    
    # Start mission
    ros2 run plan vostok1_cli --mode modular start
    
    # Stop / Resume / Reset
    ros2 run plan vostok1_cli --mode modular stop
    ros2 run plan vostok1_cli --mode modular resume
    ros2 run plan vostok1_cli --mode modular reset
    
    # Interactive mode
    ros2 run plan vostok1_cli --mode modular interactive
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import sys
import argparse
import time


class MissionCLI(Node):
    def __init__(self, mode='vostok1'):
        super().__init__('vostok1_cli')
        
        self.mode = mode
        
        # Set topic prefixes based on mode
        if mode == 'modular' or mode == 'sputnik':
            config_topic = '/sputnik/set_config'
            command_topic = '/sputnik/mission_command'
            status_topic = '/planning/mission_status'
            config_status_topic = '/sputnik/config'
            self.get_logger().info("Mode: MODULAR (Sputnik Planner)")
        else:
            config_topic = '/vostok1/set_config'
            command_topic = '/vostok1/mission_command'
            status_topic = '/vostok1/mission_status'
            config_status_topic = '/vostok1/config'
            self.get_logger().info("Mode: INTEGRATED (Vostok1)")
        
        # Publishers
        self.config_pub = self.create_publisher(String, config_topic, 10)
        self.command_pub = self.create_publisher(String, command_topic, 10)
        
        # Subscribers for status
        self.mission_status = None
        self.config_status = None
        
        self.create_subscription(
            String,
            status_topic,
            self.mission_status_callback,
            10
        )
        
        self.create_subscription(
            String,
            config_status_topic,
            self.config_callback,
            10
        )
        
        # Wait for connection
        time.sleep(0.5)
        
    def mission_status_callback(self, msg):
        try:
            self.mission_status = json.loads(msg.data)
        except:
            pass
            
    def config_callback(self, msg):
        try:
            self.config_status = json.loads(msg.data)
        except:
            pass
    
    def send_command(self, command):
        """Send mission command"""
        msg = String()
        msg.data = json.dumps({'command': command})
        self.command_pub.publish(msg)
        self.get_logger().info(f"Sent command: {command}")
        
    def send_config(self, config):
        """Send configuration"""
        msg = String()
        msg.data = json.dumps(config)
        self.config_pub.publish(msg)
        self.get_logger().info(f"Sent config: {config}")
        
    def generate_waypoints(self, lanes=8, length=50.0, width=20.0):
        """Generate waypoints with specified parameters"""
        config = {
            'lanes': lanes,
            'scan_length': length,
            'scan_width': width
        }
        self.send_config(config)
        time.sleep(0.2)
        self.send_command('generate_waypoints')
        print(f"\n✅ Waypoints generated: {lanes} lanes × {length}m length × {width}m width")
        print(f"   Estimated waypoints: {lanes * 2 - 1}")
        print(f"   Estimated distance: {length * lanes + width * (lanes - 1):.0f}m")
        
    def start_mission(self):
        """Start the mission"""
        self.send_command('start_mission')
        print("\n🚀 Mission START command sent!")
        
    def stop_mission(self):
        """Stop the mission"""
        self.send_command('stop_mission')
        print("\n🛑 Mission STOP command sent!")
        
    def resume_mission(self):
        """Resume the mission"""
        self.send_command('resume_mission')
        print("\n▶️ Mission RESUME command sent!")
        
    def reset_mission(self):
        """Reset the mission"""
        self.send_command('reset_mission')
        print("\n🔄 Mission RESET command sent!")
    
    def go_home(self):
        """Navigate back to spawn point"""
        self.send_command('go_home')
        print("\n🏠 GO HOME command sent - Returning to spawn point!")
        
    def confirm_waypoints(self):
        """Confirm waypoints"""
        self.send_command('confirm_waypoints')
        print("\n✅ Waypoints CONFIRMED!")
        
    def set_pid(self, kp=None, ki=None, kd=None):
        """Set PID parameters"""
        config = {}
        if kp is not None:
            config['kp'] = kp
        if ki is not None:
            config['ki'] = ki
        if kd is not None:
            config['kd'] = kd
        if config:
            self.send_config(config)
            print(f"\n⚙️ PID parameters updated: {config}")
            
    def set_speed(self, base=None, max_speed=None):
        """Set speed parameters"""
        config = {}
        if base is not None:
            config['base_speed'] = base
        if max_speed is not None:
            config['max_speed'] = max_speed
        if config:
            self.send_config(config)
            print(f"\n⚙️ Speed parameters updated: {config}")
            
    def show_status(self):
        """Show current status"""
        # Spin briefly to get updates
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.1)
            
        print("\n" + "=" * 50)
        print("VOSTOK1 STATUS | STATUT VOSTOK1")
        print("=" * 50)
        
        if self.mission_status:
            print(f"State: {self.mission_status.get('state', 'Unknown')}")
            print(f"Waypoint: {self.mission_status.get('waypoint', '?')}/{self.mission_status.get('total_waypoints', '?')}")
            print(f"Distance to WP: {self.mission_status.get('distance_to_waypoint', '?')}m")
        else:
            print("⚠️ No mission status received - is Vostok1 running?")
            
        if self.config_status:
            print(f"\nConfig:")
            print(f"  Lanes: {self.config_status.get('lanes', '?')}")
            print(f"  Scan Length: {self.config_status.get('scan_length', '?')}m")
            print(f"  Scan Width: {self.config_status.get('scan_width', '?')}m")
            print(f"  PID: kp={self.config_status.get('kp', '?')}, ki={self.config_status.get('ki', '?')}, kd={self.config_status.get('kd', '?')}")
            print(f"  Speed: base={self.config_status.get('base_speed', '?')}, max={self.config_status.get('max_speed', '?')}")
        
        print("=" * 50)
        
    def interactive_mode(self):
        """Interactive command mode"""
        print("\n" + "=" * 60)
        print("VOSTOK1 INTERACTIVE MODE | MODE INTERACTIF")
        print("=" * 60)
        print("Commands:")
        print("  g [lanes] [length] [width] - Generate waypoints")
        print("  c                          - Confirm waypoints")
        print("  s                          - Start mission")
        print("  x                          - Stop mission")
        print("  r                          - Resume mission")
        print("  reset                      - Reset mission")
        print("  home                       - 🏠 Go home (return to spawn)")
        print("  status                     - Show status")
        print("  pid <kp> <ki> <kd>         - Set PID")
        print("  speed <base> <max>         - Set speed")
        print("  q                          - Quit")
        print("=" * 60)
        
        while True:
            try:
                cmd = input("\nvostok1> ").strip().lower().split()
                if not cmd:
                    continue
                    
                if cmd[0] == 'q' or cmd[0] == 'quit':
                    print("Goodbye!")
                    break
                elif cmd[0] == 'g' or cmd[0] == 'generate':
                    lanes = int(cmd[1]) if len(cmd) > 1 else 8
                    length = float(cmd[2]) if len(cmd) > 2 else 50.0
                    width = float(cmd[3]) if len(cmd) > 3 else 20.0
                    self.generate_waypoints(lanes, length, width)
                elif cmd[0] == 'c' or cmd[0] == 'confirm':
                    self.confirm_waypoints()
                elif cmd[0] == 's' or cmd[0] == 'start':
                    self.start_mission()
                elif cmd[0] == 'x' or cmd[0] == 'stop':
                    self.stop_mission()
                elif cmd[0] == 'r' or cmd[0] == 'resume':
                    self.resume_mission()
                elif cmd[0] == 'reset':
                    self.reset_mission()
                elif cmd[0] == 'home':
                    self.go_home()
                elif cmd[0] == 'status':
                    self.show_status()
                elif cmd[0] == 'pid' and len(cmd) >= 4:
                    self.set_pid(float(cmd[1]), float(cmd[2]), float(cmd[3]))
                elif cmd[0] == 'speed' and len(cmd) >= 3:
                    self.set_speed(float(cmd[1]), float(cmd[2]))
                else:
                    print("Unknown command. Type 'q' to quit.")
                    
                # Brief spin to process callbacks
                rclpy.spin_once(self, timeout_sec=0.1)
                    
            except KeyboardInterrupt:
                print("\nInterrupted. Goodbye!")
                break
            except ValueError as e:
                print(f"Invalid value: {e}")
            except Exception as e:
                print(f"Error: {e}")


def main():
    parser = argparse.ArgumentParser(
        description='Vostok1 Mission CLI - Terminal control for autonomous boat',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  ros2 run plan vostok1_cli generate --lanes 8 --length 50 --width 20
  ros2 run plan vostok1_cli start
  ros2 run plan vostok1_cli stop
  ros2 run plan vostok1_cli status
  ros2 run plan vostok1_cli interactive
        """
    )
    
    subparsers = parser.add_subparsers(dest='command', help='Command to execute')
    
    # Generate command
    gen_parser = subparsers.add_parser('generate', help='Generate waypoints')
    gen_parser.add_argument('--lanes', '-l', type=int, default=8, help='Number of lanes')
    gen_parser.add_argument('--length', '-L', type=float, default=50.0, help='Lane length in meters')
    gen_parser.add_argument('--width', '-w', type=float, default=20.0, help='Lane width in meters')
    
    # Simple commands
    subparsers.add_parser('start', help='Start mission')
    subparsers.add_parser('stop', help='Stop mission')
    subparsers.add_parser('resume', help='Resume mission')
    subparsers.add_parser('reset', help='Reset mission')
    subparsers.add_parser('home', help='🏠 Go home - Return to spawn point')
    subparsers.add_parser('confirm', help='Confirm waypoints')
    subparsers.add_parser('status', help='Show status')
    subparsers.add_parser('interactive', help='Interactive mode')
    
    # PID command
    pid_parser = subparsers.add_parser('pid', help='Set PID parameters')
    pid_parser.add_argument('--kp', type=float, help='Proportional gain')
    pid_parser.add_argument('--ki', type=float, help='Integral gain')
    pid_parser.add_argument('--kd', type=float, help='Derivative gain')
    
    # Speed command
    speed_parser = subparsers.add_parser('speed', help='Set speed parameters')
    speed_parser.add_argument('--base', type=float, help='Base speed')
    speed_parser.add_argument('--max', type=float, help='Max speed')
    
    args = parser.parse_args()
    
    if not args.command:
        parser.print_help()
        return
    
    rclpy.init()
    cli = MissionCLI(mode=args.mode)
    
    try:
        if args.command == 'generate':
            cli.generate_waypoints(args.lanes, args.length, args.width)
        elif args.command == 'start':
            cli.start_mission()
        elif args.command == 'stop':
            cli.stop_mission()
        elif args.command == 'resume':
            cli.resume_mission()
        elif args.command == 'reset':
            cli.reset_mission()
        elif args.command == 'home':
            cli.go_home()
        elif args.command == 'confirm':
            cli.confirm_waypoints()
        elif args.command == 'status':
            cli.show_status()
        elif args.command == 'interactive':
            cli.interactive_mode()
        elif args.command == 'pid':
            cli.set_pid(args.kp, args.ki, args.kd)
        elif args.command == 'speed':
            cli.set_speed(args.base, args.max)
            
        # Keep alive briefly to ensure messages are sent
        time.sleep(0.3)
        
    finally:
        cli.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
