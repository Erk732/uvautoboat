# UVAutoboat Quick Start

面向新手的最小可行测试流程，按顺序执行即可跑通全流程（定位→规划→避障→到达）。

核心原理/逻辑（先理解，再动手）：
- 位姿来源：`gps_imu_pose` 用 GPS/IMU 生成本地 ENU（启动时的 GPS 为原点），frame 填 `world`。控制和目标都用这个坐标系。
- 规划：收到 goal 后，生成起点到终点的直线路径；若直线穿过危险区（hazard box），尝试侧向平移路径绕开。
- 避障：运行中融合激光/点云，软避障减速并偏转，硬避障原地转向；只有长期无进展才会后退恢复。
- 判定到达：距离目标 < `goal_tolerance`（默认 1 m）或过冲都会停推力。

## 1) 准备环境
- 终端 `cd` 到工作空间根目录（包含 `control/`）。
- 编译并刷新环境：
  ```
  colcon build --packages-select control
  source install/setup.bash
  ```
- 可选：快速自检（ROS2、关键文件等）：
  ```
  ./quick_test.sh
  ```

## 2) 启动节点
在一个终端启动一站式节点（GPS/IMU→位姿，路径规划，避障控制）：
```
ros2 launch control all_in_one_bringup.launch.py
```
看到日志提示节点启动后，保持该终端运行。
更多 launch 提示：
- 默认用 GPS/IMU 生成本地 ENU（frame=world），如需使用仿真 TF 世界坐标，可将 launch 里的 `use_tf_pose` 设为 true（确保有 `world->base_link` TF）。
- 可在 launch 中调参数，常用含义：
  - `pose_topic`：控制使用的位姿话题（默认 `/wamv/pose_filtered`）。
  - 推力/航向：`forward_thrust`、`kp_yaw`、`control_rate`。
  - 路径跟踪：`waypoint_tolerance`、`goal_tolerance`、`approach_slow_dist`、`heading_align_thresh_deg`、`overshoot_margin`。
  - 避障：`obstacle_slow_dist`、`obstacle_stop_dist`、`avoid_turn_thrust`、`avoid_diff_gain`、`avoid_clear_margin`、`avoid_max_turn_time`、`full_clear_distance`、`front_angle_deg`、`side_angle_deg`、`min_range_filter`、`vfh_*`。
  - 卡滞与恢复：`stuck_timeout`、`stuck_progress_epsilon`、`recover_reverse_time`、`recover_turn_time`、`recover_reverse_thrust`。
  - 危险区：`hazard_enabled`、`hazard_world_boxes`（世界坐标矩形，支持 `hazard_world_auto_origin` 自动用首帧位姿平移）、`plan_avoid_margin`。
  - 自动连续目标：`auto_goal_enable`、`auto_next_goals`（格式 `"x1,y1;x2,y2"`，同一坐标系下按顺序执行）。

## 3) 发送目标点
在第二个终端（已 `source install/setup.bash`）发布目标点，坐标系为本地 ENU（启动时的 GPS 原点），frame 填 `world`：
```
ros2 topic pub /planning/goal geometry_msgs/PoseStamped \
"{header: {frame_id: 'world'}, pose: {position: {x: 100.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}" --once
```
终端应打印 “New goal received …”，船开始按规划前进（先绕开已知危险区，再实时避障）。

## 3.1 自动连续目标（可选）
- 在 `control/launch/all_in_one_bringup.launch.py` 中设置：
  ```
  'auto_goal_enable': True,
  'auto_next_goals': '200,0; 250,20; 300,-10',  # 依次发布的目标列表
  ```
  坐标系与当前位姿相同（frame=world，本地 ENU）。
- 手动发布第一个目标后，节点会在抵达后按列表顺序自动发布后续目标。
- 关闭自动序列：将 `auto_goal_enable` 设为 False。

## 4) 监控运行
可选在第三个终端使用监控脚本查看位姿、推力、雷达频率等：
```
./monitor_boat.sh
```
或直接查看话题：
```
ros2 topic echo /wamv/pose_filtered --once
ros2 topic echo /planning/path
ros2 topic echo /wamv/thrusters/left/thrust --once
```

## 5) 常见问题
- 没有路径/不动：先确认 `/wamv/pose_filtered` 有数据；若无，检查 GPS/IMU 话题是否在发，必要时把 `all_in_one_stack` 的 `pose_topic` 改为 `/wamv/pose_raw` 测试。
- 目标跑偏：目标坐标必须用当前本地 ENU，frame 填 `world`；若换用仿真 TF 世界系，需要在 launch 将 `gps_imu_pose.use_tf_pose` 改为 true 并确保有 `world->base_link` TF。
- 避障过早：可在 launch 调整 `obstacle_slow_dist/stop_dist`、`min_range_filter`、`full_clear_distance`。
- 危险区绕行：hazard box 在规划阶段处理（直线穿 hazard 会侧移路径），实时避障只针对动态/未知障碍。

更多调参说明参见 `LAUNCH_TUNING_QUICK_GUIDE.txt` 和避障说明文档。***
