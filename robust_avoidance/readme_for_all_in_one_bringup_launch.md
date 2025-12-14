# UVAutoboat Quick Start

Use this walkthrough to bring up the full stack once and verify the flow (Localization → Planning → Obstacle Avoidance → Arrival).

## How the stack works

- **Pose source:** `gps_imu_pose` builds a local ENU frame (origin = first GPS fix) with `frame_id=world`. Both control and goals use this frame.
- **Planning:** A straight-line path is created to the goal; if it crosses a hazard box, the path is shifted sideways to clear it.
- **Obstacle avoidance:** Lidar/point cloud data feeds soft avoidance (slow + turn) and hard avoidance (in-place turn). Reverse is only used for recovery after prolonged lack of progress.
- **Arrival:** Thrust is cut when distance < `goal_tolerance` (default 1 m) or when the boat overshoots.

## 1) Prepare the environment

1. In a terminal, `cd` to the workspace root (the directory containing `control/`).
2. Build and source:

    ```bash
    colcon build --merge-install
    source install/setup.bash
    ```

## 2) Launch the stack

In the first terminal, start the all-in-one launch (GPS/IMU → Pose, Planning, Obstacle Avoidance, Control) and leave it running:

```bash
ros2 launch control all_in_one_bringup.launch.py
```

**Useful launch tweaks (edit `control/launch/all_in_one_bringup.launch.py`):**

- Frame source: default is GPS/IMU ENU. To use a simulation TF world frame instead, set `use_tf_pose: true` and ensure a `world->base_link` TF exists.
- Pose topic: `pose_topic` (default `/wamv/pose_filtered`; try `/wamv/pose_raw` if filtered data is missing).
- Thrust/heading: `forward_thrust`, `kp_yaw`, `control_rate`.
- Path tracking: `waypoint_tolerance`, `goal_tolerance`, `approach_slow_dist`, `heading_align_thresh_deg`, `overshoot_margin`.
- Obstacle avoidance: `obstacle_slow_dist`, `obstacle_stop_dist`, `avoid_turn_thrust`, `avoid_diff_gain`.
- Stuck/recovery: `stuck_timeout`, `stuck_progress_epsilon`, `recover_reverse_time`, `recover_turn_time`, `recover_reverse_thrust`.
- Hazard planning: `hazard_enabled`, `hazard_world_boxes`, `hazard_world_auto_origin` (shifts hazards based on first pose), `plan_avoid_margin`.
- Automatic goals: `auto_goal_enable`, `auto_next_goals` (format `"x1,y1;x2,y2"`).

## 3) Send the first goal

In a second terminal (after `source install/setup.bash`), publish a goal in the local ENU frame (`frame_id=world`):

```bash
ros2 topic pub /planning/goal geometry_msgs/PoseStamped \
"{header: {frame_id: 'world'}, pose: {position: {x: 100.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}" --once
```

You should see "New goal received..." and the boat will start: respect hazard boxes, then perform real-time avoidance as needed.

## 4) Automatic sequential goals

- Default state: enabled (`'auto_goal_enable': True` in `all_in_one_bringup.launch.py`).
- Behavior: after you send the first manual goal, subsequent goals are auto-published from `auto_next_goals` upon each arrival.
- To change: edit `auto_next_goals` (e.g., `'130,0;130,50;80,60'`) or set `auto_goal_enable` to `False` to disable.

## 5) Monitor operation

Optional third terminal:

```bash
./monitor_boat.sh
```

Or inspect topics directly:

```bash
ros2 topic echo /wamv/pose_filtered --once
ros2 topic echo /planning/path
ros2 topic echo /wamv/thrusters/left/thrust --once
```

## 6) Common issues

- **No path / not moving:** Confirm `/wamv/pose_filtered` has data; if not, check GPS/IMU publishers. For testing, you can point `pose_topic` to `/wamv/pose_raw`.
- **Goal in wrong frame:** Goals must use the current local ENU with `frame_id=world`. If you switch to TF world, set `gps_imu_pose.use_tf_pose` to `true` and ensure `world->base_link` is available.
- **Too cautious avoidance:** Tune `obstacle_slow_dist`, `obstacle_stop_dist`, `min_range_filter`, and `full_clear_distance`.
- **Hazard handling confusion:** Hazard boxes are applied in planning (path is shifted). Real-time avoidance is for dynamic/unknown obstacles only.
