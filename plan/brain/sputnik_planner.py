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
import time

from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String


class SputnikPlanner(Node):
    """
    SPUTNIK - Trajectory planning system
    (Named after the first artificial satellite)
    """
    def __init__(self):
        super().__init__('sputnik_planner_node')

        # --- PARAMETERS ---
        self.declare_parameter('scan_length', 15.0)
        self.declare_parameter('scan_width', 30.0)
        self.declare_parameter('lanes', 10)
        self.declare_parameter('waypoint_tolerance', 2.0)
        self.declare_parameter('waypoint_skip_timeout', 45.0)  # Skip waypoint if blocked for this long

        # v2.1: Hazard zone parameters (from AllInOneStack)
        self.declare_parameter('hazard_enabled', False)
        self.declare_parameter('hazard_boxes', '')  # Local frame boxes: "xmin,ymin,xmax,ymax;..."
        self.declare_parameter('hazard_world_boxes', '')  # World frame boxes
        self.declare_parameter('hazard_origin_world_x', 0.0)
        self.declare_parameter('hazard_origin_world_y', 0.0)
        self.declare_parameter('plan_avoid_margin', 5.0)  # Planning detour margin
        self.declare_parameter('hull_radius', 1.5)  # Boat hull radius for clearance

        # Get parameters
        self.scan_length = self.get_parameter('scan_length').value
        self.scan_width = self.get_parameter('scan_width').value
        self.lanes = self.get_parameter('lanes').value
        self.waypoint_tolerance = self.get_parameter('waypoint_tolerance').value
        self.waypoint_skip_timeout = self.get_parameter('waypoint_skip_timeout').value

        # v2.1: Hazard zone parameters
        self.hazard_enabled = self.get_parameter('hazard_enabled').value
        hazard_local = self._parse_hazard_boxes(str(self.get_parameter('hazard_boxes').value))
        hazard_world = self._parse_hazard_boxes(str(self.get_parameter('hazard_world_boxes').value))
        origin_wx = float(self.get_parameter('hazard_origin_world_x').value)
        origin_wy = float(self.get_parameter('hazard_origin_world_y').value)
        self.hazard_boxes = hazard_local + self._world_boxes_to_local(hazard_world, origin_wx, origin_wy)
        self.plan_avoid_margin = float(self.get_parameter('plan_avoid_margin').value)
        self.hull_radius = float(self.get_parameter('hull_radius').value)

        # --- STATE ---
        self.start_gps = None
        self.current_gps = None
        self.waypoints = []
        self.current_wp_index = 0
        self.state = "INIT"  # INIT -> WAITING_CONFIRM -> DRIVING -> FINISHED
        self.mission_start_time = None
        self.mission_armed = False  # For manual start mode
        
        # GPS initialization timeout (for worlds with slow GPS like DEFAULT)
        self.gps_init_time = None  # When node started waiting for GPS
        self.gps_timeout = 30.0  # Give GPS 30 seconds to initialize
        self.gps_timeout_warned = False  # Flag to warn once about GPS timeout
        
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
        
        # Detour request from BURAN controller
        self.create_subscription(
            String,
            '/planning/detour_request',
            self.detour_request_callback,
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
        
        # Publish mission status at 5Hz (always, not just during DRIVING)
        self.create_timer(0.2, self.publish_mission_status_timer)

        self.get_logger().info("=" * 50)
        self.get_logger().info("SPUTNIK v2.1 - Trajectory Planning System")
        self.get_logger().info("=" * 50)
        self.get_logger().info(f"Zone de balayage | Scan Area: {self.scan_length}m × {self.scan_width * self.lanes}m")
        self.get_logger().info(f"Lanes: {self.lanes}, Width: {self.scan_width}m")
        if self.hazard_enabled:
            self.get_logger().info(f"Hazard Zones: {len(self.hazard_boxes)} boxes loaded")
            self.get_logger().info(f"Planning Margin: {self.plan_avoid_margin}m, Hull: {self.hull_radius}m")
        else:
            self.get_logger().info("Hazard Zones: Disabled")
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
                    self.publish_waypoints()
                    self.publish_mission_status_timer()
                else:
                    self.get_logger().warn("GPS not available - cannot generate waypoints")
                    
            elif command == 'confirm_waypoints':
                if self.waypoints:
                    self.state = "READY"
                    self.get_logger().info("Waypoints confirmed - ready to start")
                    self.publish_mission_status_timer()
                    
            elif command == 'start_mission':
                if self.waypoints and self.state in ["READY", "WAITING_CONFIRM", "PAUSED", "FINISHED"]:
                    if self.state == "FINISHED":
                        self.current_wp_index = 0
                        # CRITICAL: Reset all home-mode and finish-related state when restarting from FINISHED
                        self.go_home_mode = False  # Clear home mode flag
                        self.detour_waypoint_inserted = False
                    self.state = "DRIVING"
                    self.mission_armed = True
                    self.mission_start_time = self.get_clock().now()
                    self.obstacle_blocking_time = 0.0
                    self.detour_waypoint_inserted = False
                    self.get_logger().info("🚀 MISSION STARTED!")
                    # Force immediate publishes so BURAN responds instantly
                    self.publish_mission_status_timer()
                    self._publish_current_target_immediate()
                else:
                    self.get_logger().warn(f"Cannot start - state={self.state}, waypoints={len(self.waypoints)}")
                    
            elif command == 'stop_mission':
                prev_state = self.state
                self.state = "PAUSED"
                self.mission_armed = False
                self.get_logger().info(f"🛑 MISSION STOPPED (was {prev_state} → now PAUSED)")
                # Force immediate status publish so BURAN stops quickly and processes state change
                self.publish_mission_status_timer()
                # Also publish once more after a brief delay to ensure BURAN receives it
                # (helps with timing issues where BURAN checks mission status between publishes)
                self.get_logger().info("📡 Publishing PAUSED state - BURAN should stop immediately")
                
            elif command == 'resume_mission':
                if self.state == "PAUSED" and self.waypoints:
                    self.state = "DRIVING"
                    self.mission_armed = True
                    # Reset obstacle blocking time for fresh start
                    self.obstacle_blocking_time = 0.0
                    self.detour_waypoint_inserted = False
                    self.get_logger().info("▶️ MISSION RESUMED")
                    # Force immediate status publish so BURAN resets and starts
                    self.publish_mission_status_timer()
                    # Force immediate target publish so BURAN has target right away
                    self._publish_current_target_immediate()
                else:
                    self.get_logger().warn(f"Cannot resume - state={self.state}, waypoints={len(self.waypoints)}")
                    
            elif command == 'cancel_waypoints':
                # Cancel waypoints, go back to init
                self.waypoints = []
                self.current_wp_index = 0
                self.state = "INIT"
                self.mission_armed = False
                self.go_home_mode = False
                self.get_logger().info("Waypoints CANCELLED")
                self.publish_mission_status_timer()
                    
            elif command == 'reset_mission':
                self.waypoints = []
                self.current_wp_index = 0
                self.state = "INIT"
                self.mission_armed = False
                self.go_home_mode = False
                self.obstacle_blocking_time = 0.0
                self.get_logger().info("🔄 MISSION RESET")
                self.publish_mission_status_timer()
                
            elif command == 'joystick_enable':
                # Enable joystick override mode
                self.state = "JOYSTICK"
                self.mission_armed = False
                self.get_logger().info("JOYSTICK MODE ENABLED")
                self.publish_mission_status_timer()
                
            elif command == 'joystick_disable':
                # Disable joystick mode
                self.state = "INIT" if not self.waypoints else "WAITING_CONFIRM"
                self.get_logger().info("JOYSTICK MODE DISABLED")
                self.publish_mission_status_timer()
            
            elif command == 'go_home':
                # Navigate back to spawn point (one-click return home)
                if self.start_gps is not None:
                    # Get current position and home position
                    if self.current_gps:
                        curr_x, curr_y = self.latlon_to_meters(self.current_gps[0], self.current_gps[1])
                    else:
                        curr_x, curr_y = 0.0, 0.0
                    home_x, home_y = self.latlon_to_meters(self.start_gps[0], self.start_gps[1])

                    # v2.1: Use hazard-aware planning for go_home
                    if self.hazard_enabled:
                        self.waypoints = self.plan_detour_around_hazard(curr_x, curr_y, home_x, home_y)
                    else:
                        self.waypoints = [(home_x, home_y)]
                    self.current_wp_index = 0
                    
                    # CRITICAL FIX: If already DRIVING, need to force state transition
                    # to reset BURAN's SASS escape mode. Set to READY first, then DRIVING.
                    if self.state == "DRIVING":
                        # Temporarily transition through READY to reset BURAN controller
                        self.state = "READY"
                        self.mission_armed = False
                        self.publish_mission_status_timer()  # Publish READY state
                        # Brief wait for BURAN to process state change
                        time.sleep(0.1)
                        self.get_logger().info("🏠 GOING HOME! (Transitioning from DRIVING)")
                    else:
                        self.get_logger().info("🏠 GOING HOME!")
                    
                    # Now transition to DRIVING
                    self.state = "DRIVING"
                    self.mission_armed = True
                    self.mission_start_time = self.get_clock().now()
                    self.go_home_mode = True  # Enable home mode for smarter obstacle handling
                    self.obstacle_blocking_time = 0.0
                    self.detour_waypoint_inserted = False
                    self.get_logger().info(f"   Destination: {self.start_gps[0]:.6f}, {self.start_gps[1]:.6f}")
                    self.get_logger().info(f"   Position locale: ({home_x:.1f}m, {home_y:.1f}m)")
                    # Publish updated waypoints
                    self.publish_waypoints()
                    # Force immediate status publish so BURAN resets escape state
                    self.publish_mission_status_timer()
                    # Force immediate target publish so BURAN has target right away
                    self._publish_current_target_immediate()
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
    
    def detour_request_callback(self, msg):
        """Handle detour waypoint request from BURAN controller"""
        import json
        try:
            data = json.loads(msg.data)
            # Support both 'x'/'y' and 'detour_x'/'detour_y' keys
            detour_x = data.get('detour_x') or data.get('x')
            detour_y = data.get('detour_y') or data.get('y')
            
            if detour_x is not None and detour_y is not None and self.state == "DRIVING":
                # Insert detour waypoint before current target
                detour_wp = {'x': detour_x, 'y': detour_y, 'is_detour': True}
                self.waypoints.insert(self.current_wp_index, detour_wp)
                self.get_logger().info(f"📍 Detour waypoint inserted at ({detour_x:.1f}, {detour_y:.1f})")
                self.publish_waypoints()
        except Exception as e:
            self.get_logger().warn(f"Detour request error: {e}")
            
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
            self.mission_armed = False  # Disarm when finished
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
        In normal mode: Skip to next waypoint after timeout or repeated obstruction.
        """
        now = self.get_clock().now()
        
        # Initialize waypoint start time
        if self.waypoint_start_time is None:
            self.waypoint_start_time = now
            self.obstacle_blocking_time = 0.0
            self.last_obstacle_check = now
            return
        
        # Track obstacle blocking time if we're getting close to waypoint
        if dist < 30.0 and self.obstacle_detected:
            if self.last_obstacle_check is not None:
                dt = (now - self.last_obstacle_check).nanoseconds / 1e9
                self.obstacle_blocking_time += dt
        else:
            # Reset blocking time if no obstacle detected at close range
            if dist < 30.0:
                self.obstacle_blocking_time = 0.0
        
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
        # Skip condition 1: Timeout exceeded (reduced timeout for faster response)
        timeout_exceeded = self.obstacle_blocking_time >= self.waypoint_skip_timeout
        
        # Skip condition 2: Obstacle detected AND we're very close (cluster of buoys)
        in_obstacle_cluster = dist < 8.0 and self.obstacle_detected
        
        if timeout_exceeded or in_obstacle_cluster:
            wp_num = self.current_wp_index + 1
            total_wp = len(self.waypoints)
            target_x, target_y = self.waypoints[self.current_wp_index]
            
            reason = "timeout" if timeout_exceeded else "obstacle_cluster"
            self.get_logger().warn(
                f"⏭️ SKIP WP {wp_num}/{total_wp} | Reason: {reason} | "
                f"Distance: {dist:.1f}m, Blocking: {self.obstacle_blocking_time:.1f}s "
                f"(target: ({target_x:.1f}, {target_y:.1f}))"
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
            f"DETOUR! Inserting detour waypoint at ({detour_x:.1f}, {detour_y:.1f})"
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
            self.get_logger().info("🏠 ARRIVED HOME!")
            self.get_logger().info("=" * 60)
            self.get_logger().info(f"Final Position: ({final_x:.1f}m, {final_y:.1f}m)")
            self.get_logger().info(f"Mission Time: {elapsed_min:.1f} minutes")
            self.get_logger().info("=" * 60)
            # Clear waypoints after go_home so user knows to regenerate
            self.waypoints = []
            self.current_wp_index = 0
            self.state = "INIT"
            self.get_logger().info("Waypoints cleared. Run 'generate' for new waypoints.")
        else:
            self.get_logger().info("✅ MISSION COMPLETE!")
            self.get_logger().info("=" * 60)
            self.get_logger().info(f"Final Position: ({final_x:.1f}m, {final_y:.1f}m)")
            self.get_logger().info(f"Waypoints: {len(self.waypoints)}")
            self.get_logger().info(f"Mission Time: {elapsed_min:.1f} minutes")
            self.get_logger().info("=" * 60)
            # Keep waypoints for potential restart
            self.get_logger().info("Run 'start' to repeat mission or 'generate' for new waypoints.")

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

        # Check GPS ready status with timeout fallback
        gps_ready = self.current_gps is not None
        
        # Initialize GPS timeout tracking
        if self.gps_init_time is None and self.current_gps is None:
            self.gps_init_time = self.get_clock().now()
        
        # GPS timeout: if no GPS after 30 seconds, assume ready anyway (fallback)
        if self.gps_init_time is not None and not gps_ready:
            gps_wait_time = (self.get_clock().now() - self.gps_init_time).nanoseconds / 1e9
            if gps_wait_time >= self.gps_timeout:
                gps_ready = True  # Fallback: consider GPS ready after timeout
                if not self.gps_timeout_warned:
                    self.get_logger().warn(
                        f"⚠️  GPS NOT RECEIVED after {self.gps_timeout}s - assuming GPS ready anyway (fallback mode)\n"
                        f"   Check: 'ros2 topic echo /wamv/sensors/gps/gps/fix --once'\n"
                        f"   Issue may be: wrong world file, Gazebo plugin problem, or VRX setup issue"
                    )
                    self.gps_timeout_warned = True

        msg = String()
        msg.data = json.dumps({
            'state': self.state,
            'current_waypoint': self.current_wp_index + 1,
            'total_waypoints': len(self.waypoints),
            'progress_percent': round(100 * self.current_wp_index / max(1, len(self.waypoints)), 1),
            'elapsed_time': round(elapsed, 1),
            'position': [round(curr_x, 2), round(curr_y, 2)],
            'mission_armed': self.mission_armed,
            'gps_ready': gps_ready,
            'joystick_override': self.state == "JOYSTICK"
        })
        self.pub_mission_status.publish(msg)

    def publish_mission_status_timer(self):
        """Timer callback to publish mission status continuously (even when not DRIVING)"""
        if self.current_gps is not None:
            curr_x, curr_y = self.latlon_to_meters(self.current_gps[0], self.current_gps[1])
        else:
            curr_x, curr_y = 0.0, 0.0
        self.publish_mission_status(curr_x, curr_y)

    def _publish_current_target_immediate(self):
        """Immediately publish current target (called on resume/go_home for instant BURAN response)"""
        if self.current_gps is None or not self.waypoints or self.current_wp_index >= len(self.waypoints):
            return

        curr_x, curr_y = self.latlon_to_meters(self.current_gps[0], self.current_gps[1])
        target_x, target_y = self.waypoints[self.current_wp_index]
        dist = math.hypot(target_x - curr_x, target_y - curr_y)
        self.publish_current_target(curr_x, curr_y, target_x, target_y, dist)
        self.get_logger().info(f"📍 Target published: ({target_x:.1f}, {target_y:.1f}) - {dist:.1f}m away")

    # ==================== v2.1: HAZARD ZONE METHODS ====================

    def _parse_hazard_boxes(self, spec: str):
        """Parse hazard box specification: "xmin,ymin,xmax,ymax;..." """
        boxes = []
        if not spec:
            return boxes
        parts = spec.split(';')
        skipped = 0
        for i, p in enumerate(parts):
            p = p.strip()
            if not p:
                continue
            vals = p.split(',')
            if len(vals) != 4:
                skipped += 1
                continue
            try:
                xmin, ymin, xmax, ymax = map(float, vals)
                if xmin > xmax:
                    xmin, xmax = xmax, xmin
                if ymin > ymax:
                    ymin, ymax = ymax, ymin
                boxes.append((xmin, ymin, xmax, ymax))
            except ValueError as e:
                self.get_logger().warn(f"Invalid hazard box #{i}: '{p}' - {e}")
                skipped += 1
                continue
        if skipped > 0:
            self.get_logger().warn(f"Skipped {skipped} invalid hazard box entries")
        return boxes

    def _world_boxes_to_local(self, boxes, origin_x: float, origin_y: float):
        """Convert world-frame hazard boxes to local ENU by subtracting origin."""
        local = []
        for xmin, ymin, xmax, ymax in boxes:
            local.append((
                xmin - origin_x,
                ymin - origin_y,
                xmax - origin_x,
                ymax - origin_y,
            ))
        return local

    def in_hazard_zone(self, x: float, y: float) -> bool:
        """Check if position is inside any hazard zone."""
        if not self.hazard_enabled or not self.hazard_boxes:
            return False
        for xmin, ymin, xmax, ymax in self.hazard_boxes:
            if xmin <= x <= xmax and ymin <= y <= ymax:
                return True
        return False

    def _segment_intersects_hazard(self, x1, y1, x2, y2, margin=0.0) -> bool:
        """Check if a line segment intersects any hazard zone (with optional margin)."""
        if not self.hazard_enabled or not self.hazard_boxes:
            return False

        def _expand_box(box, m):
            xmin, ymin, xmax, ymax = box
            return (xmin - m, ymin - m, xmax + m, ymax + m)

        def _point_in_box(px, py, box):
            xmin, ymin, xmax, ymax = box
            return xmin <= px <= xmax and ymin <= py <= ymax

        def _segments_intersect(p1, p2, p3, p4):
            def orient(a, b, c):
                return (b[0]-a[0])*(c[1]-a[1]) - (b[1]-a[1])*(c[0]-a[0])
            o1 = orient(p1, p2, p3)
            o2 = orient(p1, p2, p4)
            o3 = orient(p3, p4, p1)
            o4 = orient(p3, p4, p2)
            if o1 == 0 and o2 == 0 and o3 == 0 and o4 == 0:
                def between(a,b,c):
                    return min(a,b) <= c <= max(a,b)
                return (between(p1[0], p2[0], p3[0]) or between(p1[0], p2[0], p4[0]) or
                        between(p1[1], p2[1], p3[1]) or between(p1[1], p2[1], p4[1]))
            return (o1 * o2 <= 0) and (o3 * o4 <= 0)

        for box in self.hazard_boxes:
            exp_box = _expand_box(box, margin)
            if _point_in_box(x1, y1, exp_box) or _point_in_box(x2, y2, exp_box):
                return True
            xmin, ymin, xmax, ymax = exp_box
            corners = [(xmin, ymin), (xmax, ymin), (xmax, ymax), (xmin, ymax)]
            edges = [(corners[i], corners[(i+1) % 4]) for i in range(4)]
            for e1, e2 in edges:
                if _segments_intersect((x1, y1), (x2, y2), e1, e2):
                    return True
        return False

    def plan_detour_around_hazard(self, start_x, start_y, goal_x, goal_y):
        """
        Plan a detour path if direct segment intersects hazard zones.
        Returns list of waypoints including detours, or just [goal] if clear.
        (Ported from AllInOneStack)
        """
        margin = max(self.plan_avoid_margin, self.hull_radius * 2.0)

        # If direct path is clear, return just the goal
        if not self._segment_intersects_hazard(start_x, start_y, goal_x, goal_y, margin):
            return [(goal_x, goal_y)]

        # Calculate perpendicular direction for offset
        dx = goal_x - start_x
        dy = goal_y - start_y
        dist = math.hypot(dx, dy)
        if dist < 1e-3:
            return [(goal_x, goal_y)]

        nx = -dy / dist  # Perpendicular unit vector
        ny = dx / dist

        # Try both sides to find a clear detour
        for side in (1.0, -1.0):
            cand1 = (start_x + side * margin * nx, start_y + side * margin * ny)
            cand2 = (goal_x + side * margin * nx, goal_y + side * margin * ny)

            # Check if detour path is clear
            if (not self._segment_intersects_hazard(start_x, start_y, cand1[0], cand1[1], margin) and
                not self._segment_intersects_hazard(cand1[0], cand1[1], cand2[0], cand2[1], margin) and
                not self._segment_intersects_hazard(cand2[0], cand2[1], goal_x, goal_y, margin)):

                self.get_logger().info(
                    f"Hazard detour: {'left' if side > 0 else 'right'} offset by {margin:.1f}m"
                )
                return [cand1, cand2, (goal_x, goal_y)]

        # If no clear detour found, return direct path anyway (rely on real-time avoidance)
        self.get_logger().warn("No clear hazard detour found, using direct path")
        return [(goal_x, goal_y)]


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
