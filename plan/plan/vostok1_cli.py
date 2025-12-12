#!/usr/bin/env python3
"""
Mission CLI - Terminal-based mission control for Vostok1 / Sputnik

Use this when the web dashboard is unavailable.

MODES:
    --mode modular   (default) Use modular Sputnik planner + Buran controller
    --mode vostok1   Use integrated Vostok1 navigation

Usage (Modular - Sputnik Planner - Default):
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
    
    # Go home (return to spawn)
    ros2 run plan vostok1_cli home
    
    # Confirm waypoints
    ros2 run plan vostok1_cli confirm
    
    # Set PID parameters
    ros2 run plan vostok1_cli pid --kp 400 --ki 20 --kd 100

    # Set speed
    ros2 run plan vostok1_cli speed --base 500 --max 800

    # Generate with custom parameters (PID, speed, and turn angle)
    ros2 run plan vostok1_cli generate --kp 500 --ki 20 --kd 150 --base 400 --max 800 --max-turn 20
    
    # Show current status
    ros2 run plan vostok1_cli status
    
    # Interactive mode
    ros2 run plan vostok1_cli interactive

Usage (Integrated - Vostok1):
    # Generate waypoints
    ros2 run plan vostok1_cli --mode vostok1 generate --lanes 8 --length 15 --width 5
    
    # Start mission
    ros2 run plan vostok1_cli --mode vostok1 start
    
    # Stop / Resume / Reset
    ros2 run plan vostok1_cli --mode vostok1 stop
    ros2 run plan vostok1_cli --mode vostok1 resume
    ros2 run plan vostok1_cli --mode vostok1 reset
    
    # Interactive mode
    ros2 run plan vostok1_cli --mode vostok1 interactive
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import sys
import argparse
import time
from typing import Optional, Tuple


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
        self.buran_status = None
        
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
        
        # Also subscribe to BURAN/control status (modular mode)
        if mode == 'modular' or mode == 'sputnik':
            self.create_subscription(
                String,
                '/control/status',
                self.buran_status_callback,
                10
            )
        
        # Wait for connection
        time.sleep(0.5)

    def _spin_for_status(self, timeout: float = 1.5) -> Tuple[Optional[dict], Optional[dict]]:
        """
        Spin briefly to refresh mission/config status.
        Returns (mission_status, config_status).
        """
        start = time.time()
        while time.time() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.mission_status or self.config_status:
                break
        return self.mission_status, self.config_status

    def _waypoint_info(self) -> Tuple[int, str]:
        """
        Best-effort extraction of waypoint count and state across modes.
        Returns (count, state_string)
        """
        state = None
        count = 0
        if self.mission_status:
            state = self.mission_status.get('state', None)
            count = int(self.mission_status.get('total_waypoints', 0) or 0)
            # Sputnik exposes current_waypoint instead of waypoint index
            if count == 0 and 'current_waypoint' in self.mission_status:
                count = int(self.mission_status.get('current_waypoint') or 0)
        if self.config_status:
            state = self.config_status.get('state', state)
            if 'total_waypoints' in self.config_status:
                count = max(count, int(self.config_status.get('total_waypoints') or 0))
            if 'waypoint_count' in self.config_status:
                count = max(count, int(self.config_status.get('waypoint_count') or 0))
        return count, state or 'UNKNOWN'

    def _auto_confirm_if_needed(self, for_command='start') -> bool:
        """
        Ensure waypoints are confirmed before starting/resuming.
        Returns True if we can proceed with start/resume.
        """
        self._spin_for_status()
        count, state = self._waypoint_info()

        # Special handling for FINISHED state
        if state == "FINISHED":
            if count <= 1:
                # After go_home, only 1 waypoint (home location) - need to regenerate
                print("⚠️ Mission finished. Run 'generate' to create new waypoints.")
                return False
            # Normal FINISHED - can restart with existing waypoints
            print(f"ℹ️ Restarting from FINISHED state with {count} waypoints...")
            return True

        if count == 0:
            print("⚠️ No waypoints defined. Run 'generate' first.")
            return False

        # If already armed/running, nothing to do
        armed = False
        if self.config_status and 'mission_armed' in self.config_status:
            armed = bool(self.config_status.get('mission_armed'))
        elif self.mission_status and 'mission_armed' in self.mission_status:
            armed = bool(self.mission_status.get('mission_armed'))

        if state in ["RUNNING", "DRIVING"]:
            if for_command == 'start':
                print("ℹ️ Mission already running.")
            return armed  # Only proceed if actually armed

        # READY state - good to go
        if state == "READY":
            return True

        # PAUSED state - only allow resume, not start
        if state == "PAUSED":
            if for_command == 'start':
                print("ℹ️ Mission is PAUSED. Use 'resume' to continue or 'reset' to restart.")
                return False
            return True

        # States that still need confirmation
        needs_confirm = state in ["WAITING_CONFIRM", "WAYPOINTS_PREVIEW", "INIT", "IDLE", None, "UNKNOWN"]
        if needs_confirm:
            print("✅ Auto-confirming waypoints before start...")
            self.confirm_waypoints()
            # Give time for the nav stack to process confirmation
            self._spin_for_status(timeout=1.0)
        return True
        
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
    
    def buran_status_callback(self, msg):
        try:
            self.buran_status = json.loads(msg.data)
        except:
            pass
    
    def wait_for_ready(self, timeout=5.0):
        """Wait for navigation system to be ready"""
        print("⏳ Waiting for navigation system...")
        start = time.time()
        while time.time() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.config_status is not None:
                print("✅ Navigation system ready!")
                return True
        print("⚠️ Navigation system not responding. Is it running?")
        if self.mode == 'modular' or self.mode == 'sputnik':
            print("   Start with: ros2 launch ~/seal_ws/src/uvautoboat/launch/vostok1.launch.yaml")
        else:
            print("   Start with: ros2 run plan vostok1")
        return False
    
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
        
    def generate_waypoints(self, lanes=8, length=50.0, width=20.0,
                            kp=None, ki=None, kd=None, base_speed=None, max_speed=None, max_turn=None,
                            hazard=False, hazard_boxes=None, hazard_origin_x=None, hazard_origin_y=None,
                            astar=False, astar_hybrid=False, astar_resolution=None, astar_safety=None, astar_max=None):
        """Generate waypoints with specified parameters and optional PID/speed/turn/hazard/A* config"""
        # Wait for navigation system to be ready
        if not self.wait_for_ready():
            return False
            
        # Waypoint config (for SPUTNIK)
        config = {
            'lanes': lanes,
            'scan_length': length,
            'scan_width': width
        }
        
        # Add PID/speed/turn if specified (for BURAN - it also listens to /sputnik/set_config)
        if kp is not None:
            config['kp'] = kp
        if ki is not None:
            config['ki'] = ki
        if kd is not None:
            config['kd'] = kd
        if base_speed is not None:
            config['base_speed'] = base_speed
        if max_speed is not None:
            config['max_speed'] = max_speed
        if max_turn is not None:
            config['max_avoidance_turn_deg'] = max_turn

        # Hazard/A* options
        if hazard:
            config['hazard_enabled'] = True
        if hazard_boxes:
            config['hazard_world_boxes'] = hazard_boxes
        if hazard_origin_x is not None:
            config['hazard_origin_world_x'] = hazard_origin_x
        if hazard_origin_y is not None:
            config['hazard_origin_world_y'] = hazard_origin_y
        if astar:
            config['astar_enabled'] = True
        if astar_hybrid:
            config['astar_hybrid_mode'] = True
        if astar_resolution is not None:
            config['astar_resolution'] = astar_resolution
        if astar_safety is not None:
            config['astar_safety_margin'] = astar_safety
        if astar_max is not None:
            config['astar_max_expansions'] = astar_max
            
        self.send_config(config)
        time.sleep(0.2)
        self.send_command('generate_waypoints')
        
        print(f"\n✅ Waypoints generated: {lanes} lanes × {length}m length × {width}m width")
        print(f"   Estimated waypoints: {lanes * 2 - 1}")
        print(f"   Estimated distance: {length * lanes + width * (lanes - 1):.0f}m")
        
        # Show PID/speed if configured
        pid_info = []
        if kp is not None:
            pid_info.append(f"Kp={kp}")
        if ki is not None:
            pid_info.append(f"Ki={ki}")
        if kd is not None:
            pid_info.append(f"Kd={kd}")
        if pid_info:
            print(f"   PID: {', '.join(pid_info)}")
            
        speed_info = []
        if base_speed is not None:
            speed_info.append(f"base={base_speed}")
        if max_speed is not None:
            speed_info.append(f"max={max_speed}")
        if speed_info:
            print(f"   Speed: {', '.join(speed_info)}")
        
    def start_mission(self):
        """Start the mission"""
        if not self._auto_confirm_if_needed(for_command='start'):
            return
        self.send_command('start_mission')
        time.sleep(0.2)
        rclpy.spin_once(self, timeout_sec=0.1)
        print("\n🚀 Mission START command sent!")
        
    def stop_mission(self):
        """Stop the mission - send multiple times to ensure delivery"""
        print("\n🛑 Stopping mission...")
        # Send stop command multiple times to ensure delivery
        for i in range(3):
            self.send_command('stop_mission')
            time.sleep(0.1)
            rclpy.spin_once(self, timeout_sec=0.1)

        # Wait and verify state changed
        time.sleep(0.3)
        self._spin_for_status(timeout=1.0)
        count, state = self._waypoint_info()

        if state == "PAUSED":
            print("✅ Mission STOPPED successfully! State: PAUSED")
        else:
            print(f"⚠️ Stop command sent. Current state: {state}")
            print("   If boat is still moving, try 'stop' again or 'reset'")
        
    def resume_mission(self):
        """Resume the mission"""
        if not self._auto_confirm_if_needed(for_command='resume'):
            return
        self.send_command('resume_mission')
        time.sleep(0.2)
        rclpy.spin_once(self, timeout_sec=0.1)
        print("\n▶️ Mission RESUME command sent!")
        
    def reset_mission(self):
        """Reset the mission"""
        self.send_command('reset_mission')
        print("\n🔄 Mission RESET command sent!")
    
    def go_home(self):
        """Navigate back to spawn point"""
        # go_home sets its own waypoints, so just check if GPS is ready
        self._spin_for_status()
        gps_ready = False
        if self.config_status:
            gps_ready = self.config_status.get('gps_ready', False)
        if not gps_ready:
            print("⚠️ GPS not ready yet. Wait for GPS signal.")
            return

        self.send_command('go_home')
        time.sleep(0.2)
        rclpy.spin_once(self, timeout_sec=0.1)
        print("\n🏠 GO HOME command sent - Returning to spawn point!")
        print("   Note: After arriving home, run 'generate' to create new waypoints.")
        
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
        # Spin longer to ensure we receive status messages
        print("\n⏳ Waiting for status...")
        for _ in range(30):
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.mission_status or self.config_status:
                break
            
        print("\n" + "=" * 50)
        print("VOSTOK1 STATUS | STATUT VOSTOK1")
        print("=" * 50)
        
        # Mission status (only available during RUNNING state)
        if self.mission_status:
            state = self.mission_status.get('state', 'Unknown')
            # Support both key names (modular uses 'current_waypoint', integrated uses 'waypoint')
            waypoint = self.mission_status.get('current_waypoint', self.mission_status.get('waypoint', '?'))
            total = self.mission_status.get('total_waypoints', '?')
            progress = self.mission_status.get('progress_percent', '?')
            elapsed = self.mission_status.get('elapsed_time', '?')
            position = self.mission_status.get('position', ['?', '?'])
            
            print(f"State: {state}")
            print(f"Waypoint: {waypoint}/{total}")
            print(f"Progress: {progress}%")
            print(f"Elapsed Time: {elapsed}s")
            print(f"Position: ({position[0]}, {position[1]})")
        else:
            # Check if we at least have config (system is running but mission not active)
            if self.config_status:
                state = self.config_status.get('state', 'IDLE')
                waypoint_count = self.config_status.get('waypoint_count', 0)
                gps_ready = self.config_status.get('gps_ready', False)
                mission_armed = self.config_status.get('mission_armed', False)
                
                print(f"State: {state}")
                print(f"Waypoints: {waypoint_count} defined")
                print(f"GPS: {'✅ Ready' if gps_ready else '⏳ Waiting...'}")
                print(f"Armed: {'✅ Yes' if mission_armed else '❌ No (use confirm command)'}")
                print("\n💡 Mission status updates during RUNNING state only")
            else:
                print("⚠️ No status received")
                print("   Check if the navigation system is running:")
                if self.mode == 'modular' or self.mode == 'sputnik':
                    print("   ros2 launch ~/seal_ws/src/uvautoboat/launch/vostok1.launch.yaml")
                else:
                    print("   ros2 run plan vostok1")
        
        # Sputnik config (waypoint generation settings)
        if self.config_status:
            print(f"\nSputnik Config (Waypoints):")
            print(f"  Lanes: {self.config_status.get('lanes', '?')}")
            print(f"  Scan Length: {self.config_status.get('scan_length', '?')}m")
            print(f"  Scan Width: {self.config_status.get('scan_width', '?')}m")
            print(f"  Tolerance: {self.config_status.get('waypoint_tolerance', '?')}m")
        
        # Buran/control status (obstacle avoidance)
        if self.buran_status:
            print(f"\nBuran Status (Controller):")
            print(f"  Mode: {self.buran_status.get('mode', '?')}")
            avoidance = self.buran_status.get('avoidance_active', False)
            obstacle = self.buran_status.get('obstacle_detected', False)
            distance = self.buran_status.get('obstacle_distance', '?')
            urgency = self.buran_status.get('urgency', 0.0)
            obs_count = self.buran_status.get('obstacle_count', 0)
            is_critical = self.buran_status.get('is_critical', False)
            
            # OKO v2.0 enhanced display
            if is_critical:
                print(f"  Obstacle: 🚨 CRITICAL ({distance}m) [{obs_count} clusters]")
            elif obstacle:
                print(f"  Obstacle: ⚠️ DETECTED ({distance}m) [{obs_count} clusters]")
            else:
                print(f"  Obstacle: ✅ Clear ({distance}m)")
            
            print(f"  Urgency: {urgency*100:.0f}%")
            print(f"  Avoidance: {'🔄 Active' if avoidance else 'Inactive'}")
            print(f"  Heading: {self.buran_status.get('current_yaw', '?')}°")
        
        # Note about PID/Speed
        if self.mode == 'modular' or self.mode == 'sputnik':
            print(f"\n💡 PID/Speed are set via CLI but not broadcast by BURAN")
            print(f"   Use 'ros2 param get /buran_node kp' to check current values")
        
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
  # Generate waypoints only
  ros2 run plan vostok1_cli generate --lanes 8 --length 50 --width 20
  
  # Generate with PID and speed in one command
  ros2 run plan vostok1_cli generate --lanes 10 --length 60 --width 25 --kp 400 --ki 20 --kd 100 --base 500 --max 800
  
  # Mission control
  ros2 run plan vostok1_cli confirm
  ros2 run plan vostok1_cli start
  ros2 run plan vostok1_cli status
  ros2 run plan vostok1_cli stop
  ros2 run plan vostok1_cli home
  
  # Separate PID/speed commands
  ros2 run plan vostok1_cli pid --kp 500 --ki 25 --kd 120
  ros2 run plan vostok1_cli speed --base 600 --max 900
  
  # Interactive mode
  ros2 run plan vostok1_cli interactive
        """
    )
    
    # Global mode argument
    parser.add_argument('--mode', '-m', type=str, default='modular',
                        choices=['vostok1', 'modular', 'sputnik'],
                        help='Mode: vostok1 (integrated) or modular/sputnik (default: modular)')
    
    subparsers = parser.add_subparsers(dest='command', help='Command to execute')
    
    # Generate command with optional PID/speed
    gen_parser = subparsers.add_parser('generate', help='Generate waypoints (with optional PID/speed/hazard/A*)')
    gen_parser.add_argument('--lanes', '-l', type=int, default=8, help='Number of lanes')
    gen_parser.add_argument('--length', '-L', type=float, default=50.0, help='Lane length in meters')
    gen_parser.add_argument('--width', '-w', type=float, default=20.0, help='Lane width in meters')
    gen_parser.add_argument('--kp', type=float, help='PID Proportional gain (optional)')
    gen_parser.add_argument('--ki', type=float, help='PID Integral gain (optional)')
    gen_parser.add_argument('--kd', type=float, help='PID Derivative gain (optional)')
    gen_parser.add_argument('--base', type=float, help='Base speed in N (optional)')
    gen_parser.add_argument('--max', type=float, help='Max speed in N (optional)')
    gen_parser.add_argument('--max-turn', type=float, help='Max avoidance turn angle in degrees (optional, BURAN)')
    # Hazard/A* options (forwarded to Sputnik)
    gen_parser.add_argument('--hazard', action='store_true', help='Enable hazard avoidance (Sputnik)')
    gen_parser.add_argument('--hazard-boxes', type=str, help='Hazard world boxes string \"xmin,ymin,xmax,ymax;...\"')
    gen_parser.add_argument('--hazard-origin-x', type=float, help='Hazard origin world X')
    gen_parser.add_argument('--hazard-origin-y', type=float, help='Hazard origin world Y')
    gen_parser.add_argument('--astar', action='store_true', help='Enable runtime A* detours (Sputnik)')
    gen_parser.add_argument('--astar-hybrid', action='store_true', help='Enable A* hybrid mode (pre-plan routes)')
    gen_parser.add_argument('--astar-resolution', type=float, help='A* grid resolution (m)')
    gen_parser.add_argument('--astar-safety', type=float, help='A* safety margin (m)')
    gen_parser.add_argument('--astar-max', type=int, help='A* max expansions')
    
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
            cli.generate_waypoints(
                args.lanes, args.length, args.width,
                kp=args.kp, ki=args.ki, kd=args.kd,
                base_speed=args.base, max_speed=args.max, max_turn=args.max_turn,
                hazard=args.hazard, hazard_boxes=args.hazard_boxes,
                hazard_origin_x=args.hazard_origin_x, hazard_origin_y=args.hazard_origin_y,
                astar=args.astar, astar_hybrid=args.astar_hybrid,
                astar_resolution=args.astar_resolution,
                astar_safety=args.astar_safety, astar_max=args.astar_max
            )
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
            
        # Keep alive longer to ensure messages are sent and processed
        time.sleep(0.5)
        rclpy.spin_once(cli, timeout_sec=0.2)

    finally:
        cli.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
