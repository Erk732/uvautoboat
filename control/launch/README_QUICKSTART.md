# UVAutoboat Quick Start

This document provides a minimal viable test procedure for new users. Follow the steps in order to run the entire process (Localization -> Planning -> Obstacle Avoidance -> Arrival).

## Core Principles/Logic (Understand First, Then Execute):
-   **Pose Source:** `gps_imu_pose` generates a local ENU (East-North-Up) coordinate system using GPS/IMU data, with the initial GPS position as the origin. The frame is set to `world`. Both control and goals use this coordinate system.
-   **Planning:** Upon receiving a goal, a straight-line path is generated from the start to the destination. If the line intersects a hazard box, the path is shifted laterally to avoid it.
-   **Obstacle Avoidance:** During operation, laser/point cloud data is fused. Soft obstacle avoidance involves deceleration and turning, while hard obstacle avoidance triggers an in-place turn. The boat will only reverse to recover if there is a prolonged lack of progress.
-   **Arrival Detection:** Thrust is cut off when the distance to the goal is less than `goal_tolerance` (default 1 meter) or if the boat overshoots the target.

## 1) Prepare the Environment
-   Open a terminal and `cd` to the root of your workspace (the directory containing `control/`).
-   Build and source the environment:
    ```bash
    colcon build --merge-install
    source install/setup.bash
    ```
- 

## 2) Launch the Nodes
In one terminal, launch the all-in-one node (GPS/IMU -> Pose, Path Planning, Obstacle Avoidance Control):
```bash
ros2 launch control all_in_one_bringup.launch.py
```
Keep this terminal running after you see log messages indicating that the nodes have started.

**More Launch Tips:**
-   By default, it uses GPS/IMU to generate a local ENU (frame=world). If you need to use simulation TF world coordinates, you can set `use_tf_pose` to `true` in the launch file (ensure you have a `world->base_link` TF).
-   You can tune parameters in the launch file. Common parameters include:
    -   `pose_topic`: The pose topic used for control (default: `/wamv/pose_filtered`).
    -   Thrust/Heading: `forward_thrust`, `kp_yaw`, `control_rate`.
    -   Path Tracking: `waypoint_tolerance`, `goal_tolerance`, `approach_slow_dist`, `heading_align_thresh_deg`, `overshoot_margin`.
    -   Obstacle Avoidance: `obstacle_slow_dist`, `obstacle_stop_dist`, `avoid_turn_thrust`, `avoid_diff_gain`.
    -   Stuck & Recovery: `stuck_timeout`, `stuck_progress_epsilon`, `recover_reverse_time`, `recover_turn_time`, `recover_reverse_thrust`.
    -   Hazard Zones: `hazard_enabled`, `hazard_world_boxes` (rectangles in world coordinates, supports `hazard_world_auto_origin` to automatically translate based on the first pose), `plan_avoid_margin`.
    -   Automatic Sequential Goals: `auto_goal_enable`, `auto_next_goals` (format: `"x1,y1;x2,y2"`, executed sequentially in the same coordinate system).

## 3) Send a Goal Point
In a second terminal (after sourcing `install/setup.bash`), publish a goal point. The coordinate system is the local ENU (origin at the initial GPS position), and the frame should be set to `world`:
```bash
ros2 topic pub /planning/goal geometry_msgs/PoseStamped \
"{header: {frame_id: 'world'}, pose: {position: {x: 100.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}" --once
```
The terminal should print "New goal received...", and the boat will start moving according to the plan (first avoiding known hazard zones, then performing real-time obstacle avoidance).

## 3.1) Automatic Sequential Goals (Enabled by Default)
-   The `all_in_one_bringup.launch.py` file enables the automatic sequential goals feature by default (`'auto_goal_enable': True`).
-   After launching, once the first manual goal is sent, the node will automatically publish subsequent goals from the `auto_next_goals` list upon arrival.
-   **To Modify or Disable:**
    -   In the `control/launch/all_in_one_bringup.launch.py` file, you can modify the coordinate sequence in `auto_next_goals`.
    -   To disable this feature, change the value of `auto_goal_enable` to `False`.
    ```python
    # In control/launch/all_in_one_bringup.launch.py
    'auto_goal_enable': True,         # Change to False to disable
    # Format: "x1,y1;x2,y2"
    'auto_next_goals': '130,0;130,50;80,60', # Modify with your desired coordinates
    ```

## 4) Monitor Operation
Optionally, in a third terminal, use a monitoring script to view pose, thrust, lidar frequency, etc.:
```bash
./monitor_boat.sh
```
Or check the topics directly:
```bash
ros2 topic echo /wamv/pose_filtered --once
ros2 topic echo /planning/path
ros2 topic echo /wamv/thrusters/left/thrust --once
```

## 5) Common Issues
-   **No Path / Not Moving:** First, confirm that `/wamv/pose_filtered` has data. If not, check if the GPS/IMU topics are publishing. If necessary, change the `pose_topic` in `all_in_one_stack` to `/wamv/pose_raw` for testing.
-   **Off-track Goal:** The goal coordinates must use the current local ENU, with the frame set to `world`. If you switch to using the simulation TF world frame, you need to change `gps_imu_pose.use_tf_pose` to `true` in the launch file and ensure there is a `world->base_link` TF.
-   **Premature Obstacle Avoidance:** You can adjust `obstacle_slow_dist/stop_dist`, `min_range_filter`, and `full_clear_distance` in the launch file.
-   **Hazard Zone Avoidance:** Hazard boxes are handled during the planning phase (a path intersecting a hazard will be shifted). Real-time obstacle avoidance is only for dynamic/unknown obstacles.

