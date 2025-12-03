#!/usr/bin/env python3
"""
Sputnik Planner - GPS Waypoint Navigation Planning

Part of the modular Vostok1 architecture.
Generates lawnmower pattern waypoints, publishes navigation path.

Features:
- Lawnmower pattern generation for systematic coverage
- State machine: INIT -> WAITING_CONFIRM -> READY -> DRIVING -> FINISHED
- Waypoint timeout with skip-to-next fallback
- Return home mode with detour insertion

Topics:
    Subscribes:
        /wamv/sensors/gps/gps/fix (NavSatFix) - GPS position
    
    Publishes:
        /planning/waypoints (String) - JSON list of waypoints
        /planning/current_target (String) - JSON current target waypoint
        /planning/mission_status (String) - JSON mission state
"""

import rclpy
from rclpy.node import Node
import math
import json

from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String


class SputnikPlanner(Node):
    """
    SPUTNIK - "Satellite/Compagnon" (Référence au premier satellite artificiel)
    Système de planification de trajectoire
    """
    def __init__(self):
        super().__init__('sputnik_planner_node')

        # --- PARAMETERS ---
        self.declare_parameter('scan_length', 15.0)
        self.declare_parameter('scan_width', 30.0)
        self.declare_parameter('lanes', 10)
        self.declare_parameter('waypoint_tolerance', 2.0)
        self.declare_parameter('waypoint_skip_timeout', 45.0)  # Skip waypoint if blocked for this long

        # Get parameters
        self.scan_length = self.get_parameter('scan_length').value
        self.scan_width = self.get_parameter('scan_width').value
        self.lanes = self.get_parameter('lanes').value
        self.waypoint_tolerance = self.get_parameter('waypoint_tolerance').value
        self.waypoint_skip_timeout = self.get_parameter('waypoint_skip_timeout').value

        # --- STATE ---
        self.start_gps = None
        self.current_gps = None
        self.waypoints = []
        self.current_wp_index = 0
        self.state = "INIT"  # INIT -> WAITING_CONFIRM -> DRIVING -> FINISHED
        self.mission_start_time = None
        self.mission_armed = False  # For manual start mode
        
        # Waypoint skip tracking (for obstacles blocking waypoint)
        self.waypoint_start_time = None  # When we started trying to reach current waypoint
        self.obstacle_detected = False
        self.obstacle_blocking_time = 0.0  # Cumulative time obstacle detected near waypoint
        self.last_obstacle_check = None
        self.go_home_mode = False  # Track if we're in return-home mode
        self.home_detour_timeout = 15.0  # Insert detour after this many seconds in home mode
        self.detour_waypoint_inserted = False
        self.detour_distance = 12.0  # Distance for detour waypoints

        # --- SUBSCRIBERS ---
        self.create_subscription(
            NavSatFix,
            '/wamv/sensors/gps/gps/fix',
            self.gps_callback,
            10
        )
        
        # Mission command subscriber for CLI/dashboard control
        self.create_subscription(
            String,
            '/sputnik/mission_command',
            self.mission_command_callback,
            10
        )
        
        # Config subscriber for runtime parameter changes
        self.create_subscription(
            String,
            '/sputnik/set_config',
            self.config_callback,
            10
        )
        
        # Obstacle info subscriber for waypoint skip logic
        self.create_subscription(
            String,
            '/perception/obstacle_info',
            self.obstacle_callback,
            10
        )

        # --- PUBLISHERS ---
        self.pub_waypoints = self.create_publisher(String, '/planning/waypoints', 10)
        self.pub_current_target = self.create_publisher(String, '/planning/current_target', 10)
        self.pub_mission_status = self.create_publisher(String, '/planning/mission_status', 10)
        self.pub_config = self.create_publisher(String, '/sputnik/config', 10)

        # Control loop at 10Hz
        self.create_timer(0.1, self.planning_loop)
        
        # Publish config at 1Hz
        self.create_timer(1.0, self.publish_config)

        self.get_logger().info("=" * 50)
        self.get_logger().info("SPUTNIK - Systeme de Planification de Trajectoire")
        self.get_logger().info("Sputnik Planner - Waypoint Navigation")
        self.get_logger().info(f"Zone de balayage | Scan Area: {self.scan_length}m × {self.scan_width * self.lanes}m")
        self.get_logger().info(f"Lanes: {self.lanes}, Width: {self.scan_width}m")
        self.get_logger().info("Waiting for GPS signal...")
        self.get_logger().info("Commands: ros2 run plan vostok1_cli --help")
        self.get_logger().info("=" * 50)

    def gps_callback(self, msg):
        """Handle GPS updates"""
        self.current_gps = (msg.latitude, msg.longitude)

        # First GPS fix - store start position but wait for mission command
        if self.start_gps is None:
            self.start_gps = (msg.latitude, msg.longitude)
            self.get_logger().info(f"Base Point: {self.start_gps[0]:.6f}, {self.start_gps[1]:.6f}")
            self.get_logger().info("GPS acquired - run 'ros2 run plan vostok1_cli generate' to create waypoints")
            
    def mission_command_callback(self, msg):
        """Handle mission commands from CLI/dashboard"""
        import json
        try:
            cmd = json.loads(msg.data)
            command = cmd.get('command', '')
            
            self.get_logger().info(f"MISSION COMMAND: {command}")
            
            if command == 'generate_waypoints':
                if self.start_gps is not None:
                    self.generate_lawnmower_path()
                    self.state = "WAITING_CONFIRM"
                    self.get_logger().info(f"Waypoints generated: {len(self.waypoints)} points")
                else:
                    self.get_logger().warn("GPS not available - cannot generate waypoints")
                    
            elif command == 'confirm_waypoints':
                if self.waypoints:
                    self.state = "READY"
                    self.get_logger().info("Waypoints confirmed - ready to start")
                    
            elif command == 'start_mission':
                if self.waypoints and self.state in ["READY", "WAITING_CONFIRM", "PAUSED", "FINISHED"]:
                    if self.state == "FINISHED":
                        self.current_wp_index = 0
                    self.state = "DRIVING"
                    self.mission_armed = True
                    self.mission_start_time = self.get_clock().now()
                    self.get_logger().info("🚀 MISSION STARTED!")
                else:
                    self.get_logger().warn(f"Cannot start - state={self.state}, waypoints={len(self.waypoints)}")
                    
            elif command == 'stop_mission':
                self.state = "PAUSED"
                self.mission_armed = False
                self.get_logger().info("🛑 MISSION STOPPED")
                
            elif command == 'resume_mission':
                if self.state == "PAUSED" and self.waypoints:
                    self.state = "DRIVING"
                    self.mission_armed = True
                    self.get_logger().info("▶️ MISSION RESUMED")
                    
            elif command == 'cancel_waypoints':
                # Cancel waypoints, go back to init
                self.waypoints = []
                self.current_wp_index = 0
                self.state = "INIT"
                self.mission_armed = False
                self.get_logger().info("Waypoints ANNULÉS | CANCELLED")
                    
            elif command == 'reset_mission':
                self.waypoints = []
                self.current_wp_index = 0
                self.state = "INIT"
                self.mission_armed = False
                self.get_logger().info("🔄 MISSION RESET")
                
            elif command == 'joystick_enable':
                # Enable joystick override mode
                self.state = "JOYSTICK"
                self.mission_armed = False
                self.get_logger().info("🎮 JOYSTICK ACTIVÉ | JOYSTICK MODE ENABLED")
                
            elif command == 'joystick_disable':
                # Disable joystick mode
                self.state = "INIT" if not self.waypoints else "WAITING_CONFIRM"
                self.get_logger().info("🎮 JOYSTICK DÉSACTIVÉ | JOYSTICK MODE DISABLED")
            
            elif command == 'go_home':
                # Navigate back to spawn point (one-click return home)
                if self.start_gps is not None:
                    # Clear current waypoints and set home as only waypoint
                    home_x, home_y = self.latlon_to_meters(self.start_gps[0], self.start_gps[1])
                    self.waypoints = [(home_x, home_y)]
                    self.current_wp_index = 0
                    self.state = "DRIVING"
                    self.mission_armed = True
                    self.mission_start_time = self.get_clock().now()
                    self.go_home_mode = True  # Enable home mode for smarter obstacle handling
                    self.obstacle_blocking_time = 0.0
                    self.detour_waypoint_inserted = False
                    self.get_logger().info("🏠 RETOUR MAISON | GOING HOME!")
                    self.get_logger().info(f"   Destination: {self.start_gps[0]:.6f}, {self.start_gps[1]:.6f}")
                    self.get_logger().info(f"   Position locale: ({home_x:.1f}m, {home_y:.1f}m)")
                    # Publish updated waypoints
                    self.publish_waypoints()
                else:
                    self.get_logger().warn("Cannot go home - no spawn point recorded")
                
        except Exception as e:
            self.get_logger().error(f"Mission command error: {e}")
    
    def obstacle_callback(self, msg):
        """Track obstacle detection for waypoint skip logic"""
        import json
        try:
            data = json.loads(msg.data)
            self.obstacle_detected = data.get('obstacle_detected', False)
        except:
            pass
            
    def config_callback(self, msg):
        """Handle runtime configuration changes"""
        import json
        try:
            config = json.loads(msg.data)
            regenerate = False
            
            if 'lanes' in config:
                self.lanes = int(config['lanes'])
                regenerate = True
            if 'scan_length' in config:
                self.scan_length = float(config['scan_length'])
                regenerate = True
            if 'scan_width' in config:
                self.scan_width = float(config['scan_width'])
                regenerate = True
                
            self.get_logger().info(f"Config updated: lanes={self.lanes}, length={self.scan_length}, width={self.scan_width}")
            
        except Exception as e:
            self.get_logger().error(f"Config parse error: {e}")
            
    def publish_config(self):
        """Publish current configuration"""
        import json
        config = {
            'lanes': self.lanes,
            'scan_length': self.scan_length,
            'scan_width': self.scan_width,
            'waypoint_tolerance': self.waypoint_tolerance,
            'state': self.state,
            'waypoint_count': len(self.waypoints),
            'current_wp': self.current_wp_index,
            'gps_ready': self.start_gps is not None,
            'start_lat': self.start_gps[0] if self.start_gps else None,
            'start_lon': self.start_gps[1] if self.start_gps else None,
            'mission_armed': self.mission_armed,
            'joystick_override': self.state == "JOYSTICK"
        }
        msg = String()
        msg.data = json.dumps(config)
        self.pub_config.publish(msg)

    def latlon_to_meters(self, lat, lon):
        """Convert GPS coordinates to local meters"""
        if self.start_gps is None:
            return 0.0, 0.0
            
        R = 6371000.0  # Earth radius in meters
        d_lat = math.radians(lat - self.start_gps[0])
        d_lon = math.radians(lon - self.start_gps[1])
        lat0 = math.radians(self.start_gps[0])

        x = d_lat * R
        y = d_lon * R * math.cos(lat0)
        return y, x  # (East, North)

    def generate_lawnmower_path(self):
        """Generate zigzag lawn mower pattern waypoints"""
        self.waypoints = []

        for i in range(self.lanes):
            if i % 2 == 0:
                x_end = self.scan_length
            else:
                x_end = 0.0

            y_pos = i * self.scan_width
            self.waypoints.append((x_end, y_pos))

            if i < self.lanes - 1:
                next_y = (i + 1) * self.scan_width
                self.waypoints.append((x_end, next_y))

        self.get_logger().info(f"Generated {len(self.waypoints)} waypoints")
        
        # Publish waypoints
        self.publish_waypoints()

    def planning_loop(self):
        """Main planning loop with Vostok1-style logging"""
        if self.state != "DRIVING" or self.current_gps is None:
            return

        # Get current position
        curr_x, curr_y = self.latlon_to_meters(self.current_gps[0], self.current_gps[1])

        # Check if mission complete
        if self.current_wp_index >= len(self.waypoints):
            self.state = "FINISHED"
            self.publish_mission_status(curr_x, curr_y)
            self.finish_mission(curr_x, curr_y)
            return

        # Get target waypoint
        target_x, target_y = self.waypoints[self.current_wp_index]

        # Calculate distance to target
        dx = target_x - curr_x
        dy = target_y - curr_y
        dist = math.hypot(dx, dy)

        # Check if waypoint reached
        if dist < self.waypoint_tolerance:
            self.get_logger().info(
                f"🎯 PT {self.current_wp_index + 1}/{len(self.waypoints)} ATTEINT! | "
                f"WP REACHED! ({target_x:.1f}, {target_y:.1f})"
            )
            self.advance_to_next_waypoint()
        else:
            # Waypoint skip logic - skip if obstacle blocking for too long
            self.check_waypoint_skip(curr_x, curr_y, dist)

        # Bilingual status logging
        self.log_navigation_status(curr_x, curr_y, target_x, target_y, dist)

        # Publish current target
        self.publish_current_target(curr_x, curr_y, target_x, target_y, dist)
        
        # Publish mission status
        self.publish_mission_status(curr_x, curr_y)

    def log_navigation_status(self, curr_x, curr_y, target_x, target_y, dist):
        """Log navigation status in Vostok1 bilingual style"""
        wp_progress = f"{self.current_wp_index + 1}/{len(self.waypoints)}"
        heading = math.degrees(math.atan2(target_y - curr_y, target_x - curr_x))
        
        self.get_logger().info(
            f"PT {wp_progress} | "  # PT = Point de Trajectoire (Waypoint)
            f"Pos: ({curr_x:.1f}, {curr_y:.1f}) | "  # Pos = Position
            f"Cible: ({target_x:.1f}, {target_y:.1f}) | "  # Cible = Target
            f"Dist: {dist:.1f}m | "  # Dist = Distance
            f"Cap: {heading:.0f}°",  # Cap = Heading
            throttle_duration_sec=2.0
        )

    def advance_to_next_waypoint(self):
        """Move to next waypoint and reset skip tracking"""
        self.current_wp_index += 1
        self.waypoint_start_time = None
        self.obstacle_blocking_time = 0.0
        self.last_obstacle_check = None
        self.detour_waypoint_inserted = False  # Reset for next waypoint

    def check_waypoint_skip(self, curr_x, curr_y, dist):
        """
        Check if we should skip waypoint due to persistent obstacle blocking.
        In go_home_mode: Insert detour waypoints instead of skipping.
        In normal mode: Skip to next waypoint after timeout.
        """
        now = self.get_clock().now()
        
        # Initialize waypoint start time
        if self.waypoint_start_time is None:
            self.waypoint_start_time = now
            self.obstacle_blocking_time = 0.0
            self.last_obstacle_check = now
            return
        
        # Only track obstacle blocking time if we're close to waypoint
        if dist < 20.0 and self.obstacle_detected:
            if self.last_obstacle_check is not None:
                dt = (now - self.last_obstacle_check).nanoseconds / 1e9
                self.obstacle_blocking_time += dt
        
        self.last_obstacle_check = now
        
        # GO HOME MODE: Insert detours instead of skipping
        if self.go_home_mode:
            # Insert detour after shorter timeout (15s) to avoid circling
            if self.obstacle_blocking_time >= self.home_detour_timeout and not self.detour_waypoint_inserted:
                self.get_logger().warn(
                    f"🏠 HOME MODE: Obstacle blocking for {self.obstacle_blocking_time:.0f}s - Inserting detour"
                )
                self.insert_detour_waypoint(curr_x, curr_y)
                self.obstacle_blocking_time = 0.0  # Reset timer after inserting detour
            return  # Don't skip in home mode
        
        # NORMAL MODE: Check if we should skip
        if self.obstacle_blocking_time >= self.waypoint_skip_timeout:
            wp_num = self.current_wp_index + 1
            total_wp = len(self.waypoints)
            target_x, target_y = self.waypoints[self.current_wp_index]
            
            self.get_logger().warn(
                f"⏭️ SAUT PT {wp_num}/{total_wp} | SKIP WP - "
                f"Obstacle blocking for {self.obstacle_blocking_time:.0f}s "
                f"(target was {dist:.1f}m away at ({target_x:.1f}, {target_y:.1f}))"
            )
            self.advance_to_next_waypoint()

    def insert_detour_waypoint(self, curr_x, curr_y):
        """Insert a detour waypoint perpendicular to current heading"""
        import math
        
        # Get current heading from GPS velocity or use direction to waypoint
        if self.current_wp_index < len(self.waypoints):
            target_x, target_y = self.waypoints[self.current_wp_index]
            heading = math.atan2(target_y - curr_y, target_x - curr_x)
        else:
            heading = 0.0
        
        # Try left first (perpendicular)
        detour_angle = heading + math.pi / 2
        detour_x = curr_x + self.detour_distance * math.cos(detour_angle)
        detour_y = curr_y + self.detour_distance * math.sin(detour_angle)
        
        # Insert detour waypoint before current target
        self.waypoints.insert(self.current_wp_index, (detour_x, detour_y))
        self.detour_waypoint_inserted = True
        
        self.get_logger().warn(
            f"DÉTOUR! Inserting detour waypoint at ({detour_x:.1f}, {detour_y:.1f})"
        )
        
        # Publish updated waypoints
        self.publish_waypoints()

    def finish_mission(self, final_x, final_y):
        """Complete mission with Vostok1-style summary"""
        elapsed = 0.0
        if self.mission_start_time:
            elapsed = (self.get_clock().now() - self.mission_start_time).nanoseconds / 1e9
        elapsed_min = elapsed / 60.0
        
        was_going_home = self.go_home_mode
        self.go_home_mode = False  # Reset home mode
        
        self.get_logger().info("=" * 60)
        if was_going_home:
            self.get_logger().info("🏠 ARRIVÉ À LA MAISON! (ARRIVED HOME!)")
        else:
            self.get_logger().info("MISSION TERMINÉE! (MISSION COMPLETE!)")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"Position finale | Final Position: ({final_x:.1f}m, {final_y:.1f}m)")
        self.get_logger().info(f"Points de trajet | Waypoints: {len(self.waypoints)}")
        self.get_logger().info(f"Durée de mission | Mission Time: {elapsed_min:.1f} minutes/min")
        self.get_logger().info("=" * 60)

    def publish_waypoints(self):
        """Publish all waypoints"""
        msg = String()
        msg.data = json.dumps({
            'waypoints': self.waypoints,
            'total': len(self.waypoints)
        })
        self.pub_waypoints.publish(msg)

    def publish_current_target(self, curr_x, curr_y, target_x, target_y, dist):
        """Publish current navigation target"""
        msg = String()
        msg.data = json.dumps({
            'current_position': [round(curr_x, 2), round(curr_y, 2)],
            'target_waypoint': [round(target_x, 2), round(target_y, 2)],
            'waypoint_index': self.current_wp_index,
            'total_waypoints': len(self.waypoints),
            'distance_to_target': round(dist, 2),
            'target_heading': round(math.degrees(math.atan2(
                target_y - curr_y, target_x - curr_x
            )), 1)
        })
        self.pub_current_target.publish(msg)

    def publish_mission_status(self, curr_x, curr_y):
        """Publish mission status"""
        elapsed = 0.0
        if self.mission_start_time:
            elapsed = (self.get_clock().now() - self.mission_start_time).nanoseconds / 1e9

        msg = String()
        msg.data = json.dumps({
            'state': self.state,
            'current_waypoint': self.current_wp_index + 1,
            'total_waypoints': len(self.waypoints),
            'progress_percent': round(100 * self.current_wp_index / max(1, len(self.waypoints)), 1),
            'elapsed_time': round(elapsed, 1),
            'position': [round(curr_x, 2), round(curr_y, 2)]
        })
        self.pub_mission_status.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SputnikPlanner()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
