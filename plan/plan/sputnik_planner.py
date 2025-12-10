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
import heapq
import glob
import xml.etree.ElementTree as ET
from pathlib import Path

from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String


class AStarSolver:
    """Lightweight 8-connected A* for local detours."""
    def __init__(self, resolution=3.0, safety_margin=10.0, max_expansions=20000):
        self.resolution = resolution
        self.safety_margin = safety_margin
        self.max_expansions = max_expansions

    def world_to_grid(self, x, y, min_x, min_y):
        gx = int((x - min_x) / self.resolution)
        gy = int((y - min_y) / self.resolution)
        return gx, gy

    def grid_to_world(self, gx, gy, min_x, min_y):
        wx = (gx * self.resolution) + min_x + (self.resolution / 2.0)
        wy = (gy * self.resolution) + min_y + (self.resolution / 2.0)
        return wx, wy

    def _add_blocked_disc(self, blocked, center, radius_steps):
        cx, cy = center
        for dx in range(-radius_steps, radius_steps + 1):
            for dy in range(-radius_steps, radius_steps + 1):
                if dx * dx + dy * dy <= radius_steps * radius_steps:
                    blocked.add((cx + dx, cy + dy))

    def _add_blocked_box(self, blocked, box, min_x, min_y):
        xmin, ymin, xmax, ymax = box
        gx0, gy0 = self.world_to_grid(xmin, ymin, min_x, min_y)
        gx1, gy1 = self.world_to_grid(xmax, ymax, min_x, min_y)
        for gx in range(min(gx0, gx1), max(gx0, gx1) + 1):
            for gy in range(min(gy0, gy1), max(gy0, gy1) + 1):
                blocked.add((gx, gy))

    def plan(self, start, goal, obstacles, hazard_boxes, inflate_radius):
        """
        Plan from start->goal avoiding circular obstacles and rectangular hazards.
        obstacles: list of (x, y)
        hazard_boxes: list of (xmin, ymin, xmax, ymax) in local frame
        inflate_radius: extra meters to inflate obstacles/hazards
        """
        sx, sy = start
        gx, gy = goal

        # Bounds from start/goal/obstacles/hazards with padding
        xs = [sx, gx] + [o[0] for o in obstacles]
        ys = [sy, gy] + [o[1] for o in obstacles]
        for xmin, ymin, xmax, ymax in hazard_boxes:
            xs += [xmin, xmax]
            ys += [ymin, ymax]

        if not xs or not ys:
            return None

        pad = max(self.safety_margin * 2.0, inflate_radius * 2.0, 30.0)
        min_x = min(xs) - pad
        max_x = max(xs) + pad
        min_y = min(ys) - pad
        max_y = max(ys) + pad

        grid_w = int((max_x - min_x) / self.resolution) + 1
        grid_h = int((max_y - min_y) / self.resolution) + 1
        if grid_w <= 0 or grid_h <= 0:
            return None

        start_node = self.world_to_grid(sx, sy, min_x, min_y)
        goal_node = self.world_to_grid(gx, gy, min_x, min_y)

        blocked = set()
        # Inflate obstacles
        radius_steps = int((self.safety_margin + inflate_radius) / self.resolution)
        for ox, oy in obstacles:
            ogx, ogy = self.world_to_grid(ox, oy, min_x, min_y)
            self._add_blocked_disc(blocked, (ogx, ogy), radius_steps)

        # Block hazard boxes (already inflated when passed in)
        for box in hazard_boxes:
            self._add_blocked_box(blocked, box, min_x, min_y)

        # If start/goal blocked, fail fast
        if start_node in blocked or goal_node in blocked:
            return None

        def neighbors(node):
            x, y = node
            dirs = [(0, 1), (1, 0), (0, -1), (-1, 0),
                    (1, 1), (1, -1), (-1, 1), (-1, -1)]
            for dx, dy in dirs:
                nx, ny = x + dx, y + dy
                if 0 <= nx < grid_w and 0 <= ny < grid_h and (nx, ny) not in blocked:
                    cost = 1.414 if dx != 0 and dy != 0 else 1.0
                    yield (nx, ny), cost

        open_set = []
        heapq.heappush(open_set, (0.0, start_node))
        came_from = {}
        g_score = {start_node: 0.0}
        expansions = 0

        while open_set and expansions < self.max_expansions:
            _, current = heapq.heappop(open_set)
            expansions += 1

            if current == goal_node:
                # Reconstruct path
                path = []
                n = current
                while n in came_from:
                    wx, wy = self.grid_to_world(n[0], n[1], min_x, min_y)
                    path.append((wx, wy))
                    n = came_from[n]
                path.reverse()
                return path

            for nb, move_cost in neighbors(current):
                tentative = g_score[current] + move_cost
                if tentative < g_score.get(nb, float('inf')):
                    came_from[nb] = current
                    g_score[nb] = tentative
                    hx = nb[0] - goal_node[0]
                    hy = nb[1] - goal_node[1]
                    h = math.hypot(hx, hy)
                    heapq.heappush(open_set, (tentative + h, nb))

        return None


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
        # World metadata (for dashboards)
        self.declare_parameter('world_name', 'unknown')

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
        self.world_name = str(self.get_parameter('world_name').value)

        # v2.1: Hazard zone parameters
        self.hazard_enabled = self.get_parameter('hazard_enabled').value
        hazard_local = self._parse_hazard_boxes(str(self.get_parameter('hazard_boxes').value))
        hazard_world = self._parse_hazard_boxes(str(self.get_parameter('hazard_world_boxes').value))
        origin_wx = float(self.get_parameter('hazard_origin_world_x').value)
        origin_wy = float(self.get_parameter('hazard_origin_world_y').value)
        self.hazard_boxes = hazard_local + self._world_boxes_to_local(hazard_world, origin_wx, origin_wy)
        self.plan_avoid_margin = float(self.get_parameter('plan_avoid_margin').value)
        self.hull_radius = float(self.get_parameter('hull_radius').value)

        # v2.2: Optional A* detour planner (ported from Atlantis)
        self.declare_parameter('astar_enabled', False)
        self.declare_parameter('astar_hybrid_mode', False)  # Pre-plan A* between lawnmower waypoints
        self.declare_parameter('astar_resolution', 3.0)
        self.declare_parameter('astar_safety_margin', 12.0)
        self.declare_parameter('astar_max_expansions', 20000)
        # Pollutant source scanning (smoke generators in SDF)
        self.declare_parameter('pollutant_scan_enabled', True)
        self.declare_parameter('pollutant_sdf_glob', 'test_environment/*smoke*.sdf')
        self.astar_enabled = bool(self.get_parameter('astar_enabled').value)
        self.astar_hybrid_mode = bool(self.get_parameter('astar_hybrid_mode').value)
        self.astar = AStarSolver(
            resolution=float(self.get_parameter('astar_resolution').value),
            safety_margin=float(self.get_parameter('astar_safety_margin').value),
            max_expansions=int(self.get_parameter('astar_max_expansions').value),
        )
        self.pollutant_scan_enabled = bool(self.get_parameter('pollutant_scan_enabled').value)
        self.pollutant_sdf_glob = str(self.get_parameter('pollutant_sdf_glob').value)

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
        self.obstacle_clusters = []  # Latest OKO clusters for A* detours
        self.front_clear = float('inf')
        self.left_clear = float('inf')
        self.right_clear = float('inf')
        self.min_obstacle_distance = float('inf')
        self.obstacle_blocking_time = 0.0  # Cumulative time obstacle detected near waypoint
        self.blocked_reason = ""  # Human-readable reason for blockage
        self.declare_parameter('max_block_time', 30.0)  # Max seconds to wait before auto-detour/skip
        self.max_block_time = float(self.get_parameter('max_block_time').value)
        self.last_obstacle_check = None
        self.go_home_mode = False  # Track if we're in return-home mode
        self.home_detour_timeout = 15.0  # Insert detour after this many seconds in home mode
        self.detour_waypoint_inserted = False
        self.detour_distance = 14.0  # Lateral distance for detour waypoints
        self.detour_forward_offset = 10.0  # Forward offset when inserting side detour
        self.detour_start_time = None
        self.pollutant_sources = []  # {name, world:[x,y,z], local:[x,y]}
        self.detected_pollutants = set()  # Track which pollutants have been detected
        self.pollutant_detection_distance = 25.0  # Detection radius in meters (smoke plumes)

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
        self.pub_pollutants = self.create_publisher(String, '/perception/pollutant_sources', 10)

        # Control loop at 10Hz
        self.create_timer(0.1, self.planning_loop)
        
        # Publish config at 1Hz
        self.create_timer(1.0, self.publish_config)
        
        # Publish mission status at 5Hz (always, not just during DRIVING)
        self.create_timer(0.2, self.publish_mission_status_timer)

        self.get_logger().info("=" * 50)
        self.get_logger().info("SPUTNIK v2.2 - Trajectory Planning System")
        self.get_logger().info("=" * 50)
        self.get_logger().info(f"Zone de balayage | Scan Area: {self.scan_length}m × {self.scan_width * self.lanes}m")
        self.get_logger().info(f"Lanes: {self.lanes}, Width: {self.scan_width}m")
        if self.hazard_enabled:
            self.get_logger().info(f"Hazard Zones: {len(self.hazard_boxes)} boxes loaded")
            self.get_logger().info(f"Planning Margin: {self.plan_avoid_margin}m, Hull: {self.hull_radius}m")
        else:
            self.get_logger().info("Hazard Zones: Disabled")
        if self.astar_hybrid_mode:
            self.get_logger().info(f"A* Hybrid Mode: ENABLED (pre-plan routes between waypoints)")
        elif self.astar_enabled:
            self.get_logger().info(f"A* Runtime Detours: ENABLED (res={self.astar.resolution}m, margin={self.astar.safety_margin}m)")
        else:
            self.get_logger().info("A* Detours: Disabled")
        self.get_logger().info("Waiting for GPS signal...")
        self.get_logger().info("Commands: ros2 run plan vostok1_cli --help")
        self.get_logger().info("=" * 50)

        # Scan for pollutant sources (smoke generators) and publish
        if self.pollutant_scan_enabled:
            self.scan_pollutant_sources()
            self.publish_pollutant_sources()

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
                resumable_states = {"PAUSED", "JOYSTICK", "STOP", "STOPPED", "EMERGENCY_STOP", "WAITING_CONFIRM", "READY"}
                if self.waypoints and self.state in resumable_states:
                    self.state = "DRIVING"
                    self.mission_armed = True
                    # Reset obstacle blocking time for fresh start
                    self.obstacle_blocking_time = 0.0
                    self.detour_waypoint_inserted = False
                    self.get_logger().info(f"▶️ MISSION RESUMED from {self.state}")
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
                # If we have waypoints, drop to PAUSED so RESUME works immediately; otherwise reset to INIT
                self.state = "PAUSED" if self.waypoints else "INIT"
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
            self.min_obstacle_distance = float(data.get('min_distance', float('inf')))
            self.front_clear = float(data.get('front_clear', float('inf')))
            self.left_clear = float(data.get('left_clear', float('inf')))
            self.right_clear = float(data.get('right_clear', float('inf')))
            # Capture clusters for A* detours
            clusters = data.get('clusters', [])
            self.obstacle_clusters = [(c.get('x', 0.0), c.get('y', 0.0)) for c in clusters if 'x' in c and 'y' in c]
            if not self.obstacle_detected:
                self.detour_waypoint_inserted = False  # allow new detour once clear
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

            # v2.2: A* detour planning runtime config
            if 'astar_enabled' in config:
                self.astar_enabled = bool(config['astar_enabled'])
                self.get_logger().info(f"A* runtime detours: {'ENABLED' if self.astar_enabled else 'DISABLED'}")
            if 'astar_hybrid_mode' in config:
                self.astar_hybrid_mode = bool(config['astar_hybrid_mode'])
                self.get_logger().info(f"A* hybrid mode: {'ENABLED' if self.astar_hybrid_mode else 'DISABLED'}")
            if 'astar_resolution' in config:
                self.astar.resolution = float(config['astar_resolution'])
            if 'astar_safety_margin' in config:
                self.astar.safety_margin = float(config['astar_safety_margin'])
            if 'astar_max_expansions' in config:
                self.astar.max_expansions = int(config['astar_max_expansions'])

            # v2.1: Hazard zone runtime config
            if 'hazard_enabled' in config:
                self.hazard_enabled = bool(config['hazard_enabled'])
                self.get_logger().info(f"Hazard zone avoidance: {'ENABLED' if self.hazard_enabled else 'DISABLED'}")
            if 'hazard_origin_world_x' in config:
                self.hazard_origin_world_x = float(config['hazard_origin_world_x'])
            if 'hazard_origin_world_y' in config:
                self.hazard_origin_world_y = float(config['hazard_origin_world_y'])
            if 'hazard_world_boxes' in config:
                try:
                    hazard_world = self._parse_hazard_boxes(str(config['hazard_world_boxes']))
                    self.hazard_boxes = self._world_boxes_to_local(
                        hazard_world,
                        getattr(self, 'hazard_origin_world_x', 0.0),
                        getattr(self, 'hazard_origin_world_y', 0.0)
                    )
                    self.get_logger().info(f"Loaded {len(self.hazard_boxes)} hazard boxes from config")
                except Exception as e:
                    self.get_logger().warn(f"Failed to parse hazard_world_boxes: {e}")

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
            'joystick_override': self.state == "JOYSTICK",
            # v2.1: Hazard zone status
            'hazard_enabled': self.hazard_enabled,
            'hazard_boxes_count': len(self.hazard_boxes),
            # v2.2: A* detour planning status
            'astar_enabled': self.astar_enabled,
            'astar_hybrid_mode': self.astar_hybrid_mode,
            'astar_resolution': self.astar.resolution,
            'astar_safety_margin': self.astar.safety_margin,
            'astar_max_expansions': self.astar.max_expansions,
            # Pollutant sources
            'pollutant_sources': self.pollutant_sources,
            'world_name': self.world_name
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
        # First generate main lawnmower waypoints
        main_waypoints = []

        for i in range(self.lanes):
            if i % 2 == 0:
                x_end = self.scan_length
            else:
                x_end = 0.0

            y_pos = i * self.scan_width
            main_waypoints.append((x_end, y_pos))

            if i < self.lanes - 1:
                next_y = (i + 1) * self.scan_width
                main_waypoints.append((x_end, next_y))

        # If hybrid mode enabled, use A* to find routes between main waypoints
        if self.astar_hybrid_mode and self.start_gps is not None:
            self.get_logger().info(f"🧠 Hybrid Mode: Planning A* routes between {len(main_waypoints)} main waypoints...")
            self.waypoints = self._generate_hybrid_waypoints(main_waypoints)
            self.get_logger().info(f"✓ Hybrid generation: {len(main_waypoints)} main → {len(self.waypoints)} total waypoints")
        else:
            self.waypoints = main_waypoints
            self.get_logger().info(f"Generated {len(self.waypoints)} waypoints (simple mode)")

        # Publish waypoints
        self.publish_waypoints()

    def _generate_hybrid_waypoints(self, main_waypoints):
        """
        Generate hybrid waypoints using A* between main lawnmower points.
        Returns expanded waypoint list with A* intermediate points.
        """
        if len(main_waypoints) < 2:
            return main_waypoints

        # Get current position as start (or use 0,0 if not available)
        if self.current_gps:
            start_x, start_y = self.latlon_to_meters(self.current_gps[0], self.current_gps[1])
        else:
            start_x, start_y = 0.0, 0.0

        hybrid_path = []
        inflate = self.plan_avoid_margin + self.hull_radius

        # Inflate hazard boxes for planning
        hazard_blocks = []
        for xmin, ymin, xmax, ymax in self.hazard_boxes:
            hazard_blocks.append((
                xmin - inflate, ymin - inflate,
                xmax + inflate, ymax + inflate
            ))

        # Use obstacle clusters if available (from OKO), otherwise empty
        obstacles = self.obstacle_clusters if hasattr(self, 'obstacle_clusters') and self.obstacle_clusters else []

        # Plan from start to first waypoint
        first_wp = main_waypoints[0]
        path_segment = self.astar.plan(
            (start_x, start_y),
            first_wp,
            obstacles,
            hazard_blocks,
            inflate_radius=inflate
        )

        if path_segment:
            # Add A* path, excluding start position
            for pt in path_segment:
                if math.hypot(pt[0] - start_x, pt[1] - start_y) > 1.0:
                    hybrid_path.append(pt)

        # Always include first main waypoint
        hybrid_path.append(first_wp)

        # Plan between consecutive main waypoints
        for i in range(len(main_waypoints) - 1):
            wp_start = main_waypoints[i]
            wp_end = main_waypoints[i + 1]

            # Try A* planning between waypoints
            path_segment = self.astar.plan(
                wp_start,
                wp_end,
                obstacles,
                hazard_blocks,
                inflate_radius=inflate
            )

            if path_segment and len(path_segment) > 0:
                # A* found a detour - add intermediate points
                for pt in path_segment:
                    # Skip if too close to start waypoint
                    if math.hypot(pt[0] - wp_start[0], pt[1] - wp_start[1]) > 1.0:
                        hybrid_path.append(pt)

            # Always add the end main waypoint
            hybrid_path.append(wp_end)

        return hybrid_path

    def planning_loop(self):
        """Main planning loop with Vostok1-style logging"""
        if self.state != "DRIVING" or self.current_gps is None:
            return

        # Get current position
        curr_x, curr_y = self.latlon_to_meters(self.current_gps[0], self.current_gps[1])

        # Check for pollutant sources nearby
        self.detect_pollutant_sources(curr_x, curr_y)

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

        # Allow looser tolerance when returning home to avoid lingering near home
        effective_tol = self.waypoint_tolerance
        if self.go_home_mode and (self.current_wp_index >= len(self.waypoints) - 1):
            effective_tol = max(self.waypoint_tolerance, 5.0)

        # Opportunistic side detour for close obstacles (buoy clusters)
        if (self.obstacle_detected and not self.detour_waypoint_inserted and
                self.front_clear < 8.0 and self.min_obstacle_distance < 10.0):
            heading = math.atan2(dy, dx)
            side = 'left' if self.left_clear > self.right_clear else 'right'
            self.insert_side_detour(curr_x, curr_y, heading, side)

        # Check if waypoint reached
        if dist < effective_tol:
            self.get_logger().info(
                f"🎯 PT {self.current_wp_index + 1}/{len(self.waypoints)} ATTEINT! | "
                f"WP REACHED! ({target_x:.1f}, {target_y:.1f})"
            )
            if self.detour_waypoint_inserted:
                # Clear detour state when any waypoint is reached
                self.detour_waypoint_inserted = False
                self.detour_start_time = None
            self.advance_to_next_waypoint()
        else:
            # If stuck on a detour too long, skip it
            if self.detour_waypoint_inserted and self.detour_start_time is not None:
                elapsed = (self.get_clock().now() - self.detour_start_time).nanoseconds / 1e9
                if elapsed > 15.0:  # seconds
                    self.get_logger().warn(
                        f"⏭️ Detour timeout ({elapsed:.1f}s) - skipping detour waypoint"
                    )
                    self.detour_waypoint_inserted = False
                    self.detour_start_time = None
                    self.advance_to_next_waypoint()
                    # Recompute distance after skipping
                    if self.current_wp_index < len(self.waypoints):
                        target_x, target_y = self.waypoints[self.current_wp_index]
                        dx = target_x - curr_x
                        dy = target_y - curr_y
                        dist = math.hypot(dx, dy)

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
        self.blocked_reason = ""
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
                self.blocked_reason = "obstacle_persistent"
        else:
            # Reset blocking time if no obstacle detected at close range
            if dist < 30.0:
                self.obstacle_blocking_time = 0.0
                self.blocked_reason = ""
        
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
                self.blocked_reason = "detour_home"
            return  # Don't skip in home mode
        
        # NORMAL MODE: Check if we should skip
        # If blocked too long, try a side detour automatically
        if self.obstacle_blocking_time >= self.max_block_time and not self.detour_waypoint_inserted:
            self.get_logger().warn(
                f"⏳ Obstacle blocking for {self.obstacle_blocking_time:.0f}s - inserting detour waypoint"
            )
            self.insert_detour_waypoint(curr_x, curr_y)
            self.obstacle_blocking_time = 0.0
            self.blocked_reason = "detour_auto"
            return

        # Skip condition 1: Timeout exceeded (reduced timeout for faster response)
        timeout_exceeded = self.obstacle_blocking_time >= self.waypoint_skip_timeout
        
        # Skip condition 2: Obstacle detected AND we're very close (cluster of buoys)
        in_obstacle_cluster = dist < 8.0 and self.obstacle_detected
        
        if timeout_exceeded or in_obstacle_cluster:
            wp_num = self.current_wp_index + 1
            total_wp = len(self.waypoints)
            target_x, target_y = self.waypoints[self.current_wp_index]

            # Try A* detour before skipping
            if self.astar_enabled:
                detour_path = self.plan_astar_detour(curr_x, curr_y, target_x, target_y)
                if detour_path:
                    self.get_logger().info(
                        f"🧭 A* DETOUR inserted for WP {wp_num}/{total_wp} ({len(detour_path)} segments)"
                    )
                    self.obstacle_blocking_time = 0.0
                    self.blocked_reason = "detour_astar"
                    return
            
            reason = "timeout" if timeout_exceeded else "obstacle_cluster"
            self.get_logger().warn(
                f"⏭️ SKIP WP {wp_num}/{total_wp} | Reason: {reason} | "
                f"Distance: {dist:.1f}m, Blocking: {self.obstacle_blocking_time:.1f}s "
                f"(target: ({target_x:.1f}, {target_y:.1f}))"
            )
            self.blocked_reason = "skipped_" + reason
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

    def insert_side_detour(self, curr_x, curr_y, heading, side='left'):
        """Insert a perpendicular + forward detour based on obstacle side."""
        import math
        lateral = self.detour_distance
        forward = self.detour_forward_offset
        angle = heading + (math.pi / 2 if side == 'left' else -math.pi / 2)
        detour_x = curr_x + lateral * math.cos(angle) + forward * math.cos(heading)
        detour_y = curr_y + lateral * math.sin(angle) + forward * math.sin(heading)
        self.waypoints.insert(self.current_wp_index, (detour_x, detour_y))
        self.detour_waypoint_inserted = True
        self.detour_start_time = self.get_clock().now()
        self.get_logger().warn(
            f"📍 Side detour ({side.upper()}): ({detour_x:.1f}, {detour_y:.1f}) | "
            f"Front={self.front_clear:.1f}m L={self.left_clear:.1f}m R={self.right_clear:.1f}m"
        )
        self.publish_waypoints()

    def plan_astar_detour(self, curr_x, curr_y, target_x, target_y):
        """Run A* to replace the current leg with a detour path."""
        # Inflate hazards by margin+hull to keep clearance
        inflate = self.plan_avoid_margin + self.hull_radius
        hazard_blocks = []
        for xmin, ymin, xmax, ymax in self.hazard_boxes:
            hazard_blocks.append((
                xmin - inflate, ymin - inflate,
                xmax + inflate, ymax + inflate
            ))

        obstacles = self.obstacle_clusters if self.obstacle_clusters else []
        path = self.astar.plan(
            (curr_x, curr_y),
            (target_x, target_y),
            obstacles,
            hazard_blocks,
            inflate_radius=inflate
        )

        if not path:
            return None

        # Replace current waypoint with the planned segments (include goal)
        # Ensure we don't include the current position as a waypoint
        filtered = [(x, y) for (x, y) in path if math.hypot(x - curr_x, y - curr_y) > 1.0]
        if not filtered:
            return None

        self.waypoints[self.current_wp_index:self.current_wp_index + 1] = filtered
        self.publish_waypoints()
        self.detour_waypoint_inserted = True
        return filtered

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
            'detour_active': self.detour_waypoint_inserted,
            'joystick_override': self.state == "JOYSTICK",
            'blocked_reason': self.blocked_reason
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

    # ==================== POLLUTANT SOURCE SCAN (SMOKE GENERATORS) ====================
    def scan_pollutant_sources(self):
        """Parse SDF files to find smoke generators and cache their positions."""
        sources = []
        # Locate repo root containing test_environment
        base = Path(__file__).resolve()
        roots = []
        # Walk up to find test_environment when running from source
        for _ in range(8):
            if (base / 'test_environment').exists():
                roots.append(base)
                break
            base = base.parent
        # Also try CWD and workspace-style path
        roots.append(Path.cwd())
        roots.append(Path.cwd() / 'test_environment')

        patterns = [p.strip() for p in self.pollutant_sdf_glob.split(';') if p.strip()]
        sdf_files = []
        for pat in patterns:
            # Absolute glob
            if pat.startswith('/'):
                sdf_files.extend(glob.glob(pat))
                continue
            # Relative glob against candidate roots
            for root in roots:
                if root is None:
                    continue
                sdf_files.extend(glob.glob(str(root / pat)))

        for sdf in sdf_files:
            try:
                tree = ET.parse(sdf)
                root_elem = tree.getroot()
                for model in root_elem.findall('.//model'):
                    name = model.get('name', '')
                    if 'smoke' not in name.lower():
                        continue
                    pose_elem = model.find('pose')
                    if pose_elem is None or pose_elem.text is None:
                        continue
                    pose_vals = [float(x) for x in pose_elem.text.strip().split()]
                    wx, wy = pose_vals[0], pose_vals[1]
                    wz = pose_vals[2] if len(pose_vals) > 2 else 0.0
                    # Default to world coords as local; apply hazard-origin offset if available
                    lx, ly = wx, wy
                    try:
                        hx = float(self.get_parameter('hazard_origin_world_x').value)
                        hy = float(self.get_parameter('hazard_origin_world_y').value)
                        lx = wx - hx
                        ly = wy - hy
                    except Exception:
                        pass
                    sources.append({
                        'name': name,
                        'world': [round(wx, 2), round(wy, 2), round(wz, 2)],
                        'local': [round(lx, 2), round(ly, 2)]
                    })
            except Exception as e:
                self.get_logger().warn(f"Pollutant scan failed for {sdf}: {e}")

        self.pollutant_sources = sources
        if sources:
            self.get_logger().info("=" * 70)
            self.get_logger().info(f"🌫️ POLLUTANT SOURCES DETECTED | SOURCES DE POLLUANTS DÉTECTÉES: {len(sources)}")
            self.get_logger().info("=" * 70)
            for i, s in enumerate(sources, 1):
                self.get_logger().info(
                    f"  [{i}] {s['name']}: World({s['world'][0]:7.1f}, {s['world'][1]:7.1f})m → "
                    f"Local({s['local'][0]:7.1f}, {s['local'][1]:7.1f})m"
                )
            self.get_logger().info("=" * 70)
        else:
            self.get_logger().info("=" * 70)
            self.get_logger().info("✅ NO POLLUTANT SOURCES | AUCUNE SOURCE DE POLLUANT DÉTECTÉE")
            self.get_logger().info("   (World loaded without smoke generators | Monde chargé sans générateurs de fumée)")
            self.get_logger().info("=" * 70)

    def publish_pollutant_sources(self):
        """Publish pollutant sources for dashboard/minimap."""
        # Always publish count, even if no sources found
        msg = String()
        msg.data = json.dumps({
            'sources': self.pollutant_sources,
            'count': len(self.pollutant_sources),
            'status': 'detected' if self.pollutant_sources else 'none'
        })
        self.pub_pollutants.publish(msg)

    def detect_pollutant_sources(self, curr_x, curr_y):
        """
        Check if boat is near any pollutant source (smoke generator).
        Log detection when boat comes within detection_distance of a source.
        """
        if not self.pollutant_sources:
            return

        for source in self.pollutant_sources:
            source_name = source.get('name', 'Unknown')
            local_pos = source.get('local') or source.get('world') or [0, 0]
            source_x, source_y = local_pos[0], local_pos[1]

            # Calculate distance to this pollutant source
            distance = math.hypot(curr_x - source_x, curr_y - source_y)

            # Check if detected and log if within range
            if distance <= self.pollutant_detection_distance:
                if source_name not in self.detected_pollutants:
                    # First detection - log it
                    self.get_logger().warn(
                        f"🌫️ POLLUTANT SOURCE DETECTED | SOURCE DE POLLUANT DÉTECTÉE: "
                        f"{source_name} at ({source_x:.1f}m, {source_y:.1f}m), "
                        f"Distance: {distance:.2f}m"
                    )
                    self.detected_pollutants.add(source_name)
            else:
                # Clear detection if we move away
                if source_name in self.detected_pollutants:
                    # Check if we've moved far enough away to reset detection
                    if distance > self.pollutant_detection_distance + 2.0:  # Hysteresis
                        self.detected_pollutants.discard(source_name)

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
