# --- NEW LIDAR SUBSCRIBER ---
from sensor_msgs.msg import LaserScan

class Apollo11(Node):
    def __init__(self):
        super().__init__('apollo11_node')
        # ... your existing __init__ code ...

        # --- OBSTACLE AVOIDANCE SETTINGS ---
        self.avoidance_distance = 12.0  # meters, trigger only when very close
        self.avoid_duration = 1.5       # seconds to turn
        self.obstacle_state = 0          # -1 = avoid right, +1 = avoid left, 0 = clear
        self.avoid_start_time = 0.0

        # --- NEW LIDAR SUBSCRIBER ---
        self.sub_lidar = self.create_subscription(
            LaserScan,
            '/wamv/sensors/lidars/lidar_wamv_sensor/scan',
            self.lidar_callback,
            10
        )

    # --- NEW LIDAR CALLBACK ---
    def lidar_callback(self, msg: LaserScan):
        # Check front sector: ±45°
        num_points = len(msg.ranges)
        front_indices = range(num_points // 2 - num_points // 8, num_points // 2 + num_points // 8)

        # Get front distances
        front_ranges = [msg.ranges[i] for i in front_indices if not math.isinf(msg.ranges[i])]

        if not front_ranges:
            self.obstacle_state = 0
            return

        min_front = min(front_ranges)
        if min_front < self.avoidance_distance:
            # Obstacle detected, decide which side is freer
            left_indices  = range(num_points // 2, num_points // 2 + num_points // 8)
            right_indices = range(num_points // 2 - num_points // 8, num_points // 2)

            left_avg  = sum([msg.ranges[i] for i in left_indices if not math.isinf(msg.ranges[i])]) / max(1, len(left_indices))
            right_avg = sum([msg.ranges[i] for i in right_indices if not math.isinf(msg.ranges[i])]) / max(1, len(right_indices))

            self.obstacle_state = 1 if left_avg > right_avg else -1
            self.avoid_start_time = time.time()
            self.state = "AVOIDING"
        else:
            self.obstacle_state = 0

    # --- MODIFY CONTROL LOOP: ADD AVOIDING STATE ---
    def control_loop(self):
        if self.current_gps is None or self.state == "INIT": 
            return

        curr_x, curr_y = self.latlon_to_meters(self.current_gps[0], self.current_gps[1])

        # --- RECOVERY LOGIC (unchanged) ---
        if self.state == "REVERSING":
            self.send_thrust(-1000.0, -1000.0)
            if (time.time() - self.recovery_start_time) > 10.0:
                self.state = "TURNING"
                self.recovery_start_time = time.time()
            return

        if self.state == "TURNING":
            self.send_thrust(800.0, -800.0)
            if (time.time() - self.recovery_start_time) > 2.0:
                self.get_logger().info("Recovery Complete. Resuming Mission.")
                self.state = "DRIVING"
                self.last_check_time = time.time()
                self.last_check_pos = (curr_x, curr_y)
            return

        # --- NEW OBSTACLE AVOIDANCE ---
        if self.state == "AVOIDING":
            turn_power = 600.0  # fixed turning thrust
            if self.obstacle_state == 1:   # more space on left → turn left
                self.send_thrust(-turn_power, turn_power)
            elif self.obstacle_state == -1:  # more space on right → turn right
                self.send_thrust(turn_power, -turn_power)
            else:
                self.send_thrust(0.0, 0.0)

            # Exit avoidance after duration
            if (time.time() - self.avoid_start_time) > self.avoid_duration:
                self.state = "DRIVING"
            return

        # --- DRIVING LOGIC (unchanged) ---
        if self.state == "DRIVING":
            self.check_if_stuck(curr_x, curr_y)
            if self.current_wp_index >= len(self.waypoints):
                self.stop_boat()
                self.get_logger().info("Mission Complete.")
                self.state = "FINISHED"
                return

            target_x, target_y = self.waypoints[self.current_wp_index]
            dx = target_x - curr_x
            dy = target_y - curr_y
            dist = math.hypot(dx, dy)

            if dist < 4.0:
                self.get_logger().info(f"Waypoint {self.current_wp_index + 1} Reached")
                self.current_wp_index += 1
                return

            target_angle = math.atan2(dy, dx)
            angle_error = target_angle - self.current_yaw
            while angle_error > math.pi: angle_error -= 2 * math.pi
            while angle_error < -math.pi: angle_error += 2 * math.pi

            kp = 300.0
            turn_power = max(-800.0, min(800.0, angle_error * kp))

            left = self.speed - turn_power
            right = self.speed + turn_power
            left = max(-1000.0, min(1000.0, left))
            right = max(-1000.0, min(1000.0, right))

            self.send_thrust(left, right)
