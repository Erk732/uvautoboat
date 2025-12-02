#!/usr/bin/env python3
"""
Planner Node - GPS Waypoint Navigation Planning

Part of the modular Vostok1 architecture.
Generates lawnmower pattern waypoints, publishes navigation path.

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
    SPUTNIK (СПУТНИК) - "Satellite/Companion" in Russian
    First artificial satellite reference
    """
    def __init__(self):
        super().__init__('sputnik_planner_node')

        # --- PARAMETERS ---
        self.declare_parameter('scan_length', 15.0)
        self.declare_parameter('scan_width', 30.0)
        self.declare_parameter('lanes', 10)
        self.declare_parameter('waypoint_tolerance', 2.0)

        # Get parameters
        self.scan_length = self.get_parameter('scan_length').value
        self.scan_width = self.get_parameter('scan_width').value
        self.lanes = self.get_parameter('lanes').value
        self.waypoint_tolerance = self.get_parameter('waypoint_tolerance').value

        # --- STATE ---
        self.start_gps = None
        self.current_gps = None
        self.waypoints = []
        self.current_wp_index = 0
        self.state = "INIT"  # INIT -> WAITING_CONFIRM -> DRIVING -> FINISHED
        self.mission_start_time = None
        self.mission_armed = False  # For manual start mode

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
        self.get_logger().info("СПУТНИК (SPUTNIK) - Система Планирования Маршрута")
        self.get_logger().info("Sputnik Planner - Waypoint Navigation")
        self.get_logger().info(f"Зона сканирования | Scan Area: {self.scan_length}m × {self.scan_width * self.lanes}m")
        self.get_logger().info(f"Lanes: {self.lanes}, Width: {self.scan_width}m")
        self.get_logger().info("Waiting for GPS signal...")
        self.get_logger().info("Commands: ros2 run plan mission_cli --help")
        self.get_logger().info("=" * 50)

    def gps_callback(self, msg):
        """Handle GPS updates"""
        self.current_gps = (msg.latitude, msg.longitude)

        # First GPS fix - store start position but wait for mission command
        if self.start_gps is None:
            self.start_gps = (msg.latitude, msg.longitude)
            self.get_logger().info(f"Base Point: {self.start_gps[0]:.6f}, {self.start_gps[1]:.6f}")
            self.get_logger().info("GPS acquired - run 'ros2 run plan mission_cli generate' to create waypoints")
            
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
                    
            elif command == 'reset_mission':
                self.waypoints = []
                self.current_wp_index = 0
                self.state = "INIT"
                self.mission_armed = False
                self.get_logger().info("🔄 MISSION RESET")
                
        except Exception as e:
            self.get_logger().error(f"Mission command error: {e}")
            
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
            'current_wp': self.current_wp_index
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
                f"🎯 ТМ {self.current_wp_index + 1}/{len(self.waypoints)} ДОСТИГНУТА! | "
                f"WP REACHED! ({target_x:.1f}, {target_y:.1f})"
            )
            self.current_wp_index += 1

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
            f"ТМ {wp_progress} | "  # ТМ = Точка Маршрута (Waypoint)
            f"Поз: ({curr_x:.1f}, {curr_y:.1f}) | "  # Поз = Позиция (Position)
            f"Цель: ({target_x:.1f}, {target_y:.1f}) | "  # Цель = Target
            f"Дист: {dist:.1f}m | "  # Дист = Дистанция (Distance)
            f"Курс: {heading:.0f}°",  # Курс = Heading
            throttle_duration_sec=2.0
        )

    def finish_mission(self, final_x, final_y):
        """Complete mission with Vostok1-style summary"""
        elapsed = 0.0
        if self.mission_start_time:
            elapsed = (self.get_clock().now() - self.mission_start_time).nanoseconds / 1e9
        elapsed_min = elapsed / 60.0
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("МИССИЯ ЗАВЕРШЕНА! (MISSION COMPLETE!)")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"Конечная позиция | Final Position: ({final_x:.1f}m, {final_y:.1f}m)")
        self.get_logger().info(f"Точек маршрута | Waypoints: {len(self.waypoints)}")
        self.get_logger().info(f"Время миссии | Mission Time: {elapsed_min:.1f} минут/min")
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
