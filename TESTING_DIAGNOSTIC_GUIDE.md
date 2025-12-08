# 系统测试与诊断完整指南

## 📋 测试清单

### 第一阶段：基础系统验证（5分钟）

#### 1.1 确认所有节点都在运行

```bash
# 启动系统
ros2 launch control all_in_one_bringup.launch.py

# 在另一个终端，查看运行的节点
ros2 node list
```

**应该看到的输出：**
```
/gps_imu_pose
/pose_filter
/all_in_one_stack
/gazebo              # 如果在仿真
```

**如果节点缺少：**
- ❌ `/gps_imu_pose` 缺少 → GPS/IMU 数据无法获取，检查传感器
- ❌ `/pose_filter` 缺少 → 位置估计失败
- ❌ `/all_in_one_stack` 缺少 → 控制系统启动失败，查看日志

#### 1.2 验证关键话题是否活跃

```bash
# 检查话题列表
ros2 topic list | grep -E "pose|lidar|goal|thrust"

# 应该看到的话题：
/wamv/pose_raw           # GPS/IMU 原始位置
/wamv/pose_filtered      # 滤波后的位置
/wamv/sensors/lidars/*/scan          # 激光雷达扫描
/wamv/sensors/lidars/*/points        # 激光雷达点云
/planning/goal           # 目标位置（输入）
/planning/path           # 规划路径（输出）
/wamv/thrusters/left/thrust          # 左推进器命令
/wamv/thrusters/right/thrust         # 右推进器命令
```

**诊断命令：**

```bash
# 检查位置数据是否更新
ros2 topic hz /wamv/pose_filtered
# 应该显示 ~20 Hz 的频率

# 检查激光雷达数据
ros2 topic hz /wamv/sensors/lidars/lidar_wamv_sensor/scan
# 应该显示 ~10-20 Hz 的频率

# 实时查看位置
ros2 topic echo /wamv/pose_filtered | head -5
# 应该看到 x, y, z 坐标在变化
```

---

### 第二阶段：避障恢复功能测试（10分钟）

#### 2.1 测试场景：船应该避开障碍物后继续走

```bash
# 终端1：启动系统
ros2 launch control all_in_one_bringup.launch.py

# 终端2：发送一个远处的目标（比如 100m 外）
ros2 topic pub /planning/goal geometry_msgs/PoseStamped \
  "{header: {frame_id: 'world'}, pose: {position: {x: 100.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}" --once

# 终端3：监看日志输出，看船是否：
# a) 朝向目标移动 ✅
# b) 遇到障碍物时转向避开 ✅
# c) 避开后立即恢复朝向目标 ✅（这是关键！）

ros2 launch control all_in_one_bringup.launch.py 2>&1 | grep -E "goal|avoid|recover|stuck"
```

**预期日志输出：**

```
[all_in_one_stack]: Goal received at (100, 0)
[all_in_one_stack]: Waypoint 0 distance: 95.3 m
[all_in_one_stack]: Lidar mins: front=8.5m, left=12.0m, right=11.0m
[all_in_one_stack]: Force avoidance active, turning left
[all_in_one_stack]: Exiting force avoidance, resuming navigation
[all_in_one_stack]: Waypoint 0 distance: 60.2 m
```

#### 2.2 关键观察点

**观察 1：避障是否立即触发？**

```bash
# 查看激光雷达检测
ros2 topic echo /wamv/sensors/lidars/lidar_wamv_sensor/scan

# 如果 range 中有很多小于 8.0 的值，说明检测到障碍物
```

**观察 2：避障转向是否有效？**

```bash
# 查看推进器命令
ros2 topic echo /wamv/thrusters/left/thrust
ros2 topic echo /wamv/thrusters/right/thrust

# 避障时，应该看到 left_thrust 和 right_thrust 差异很大
# 例如：left=100, right=400 (右转)
```

**观察 3：避障后是否恢复导航？**

```bash
# 监看距离是否继续减小
watch -n 0.5 'ros2 topic echo /planning/path | grep -A 20 "poses:"'

# 或者通过目标距离判断
# distance 应该不断减小：100 → 95 → 90 → 85 ...
# 如果距离卡住（不变），说明避障后没有恢复导航
```

---

### 第三阶段：坐标系验证（5分钟）

#### 3.1 确认目标坐标是相对于船的起始位置

```bash
# 终端1：启动系统，记下船的初始位置
ros2 launch control all_in_one_bringup.launch.py 2>&1 | grep "pose"

# 终端2：等船稳定后，查看当前位置
sleep 5
ros2 topic echo /wamv/pose_filtered | head -10

# 记下初始位置，比如：x = -0.975, y = 25.282
```

**坐标系测试：**

```bash
# 目标应该相对于初始位置计算
# 如果船在 (-0.975, 25.282)，目标 (50, 0) 表示：
#   - 向东（+X）50米
#   - 向北（+Y）不动

# 发送一个目标：100m 向东
ros2 topic pub /planning/goal geometry_msgs/PoseStamped \
  "{header: {frame_id: 'world'}, pose: {position: {x: 100.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}" --once

# 观察船是否向东移动
# 如果船向西移动，说明坐标系反了！
```

#### 3.2 验证 ENU 坐标系

```bash
# 检查 GPS 原点
ros2 param get /gps_imu_pose origin_lat
ros2 param get /gps_imu_pose origin_lon

# 应该看到类似：
# Float value is: 48.263
# Float value is: -122.424

# 这是 ENU 原点的 GPS 坐标
```

---

### 第四阶段：参数调试（可选）

如果避障后不恢复，尝试调整这些参数：

```yaml
# 在 all_in_one_bringup.launch.py 中修改：

# 降低避障触发距离（更激进的避障）
'full_clear_distance': 15.0      # 从 20.0 改到 15.0

# 降低停止距离（更早开始恢复）
'obstacle_stop_dist': 5.0        # 从 6.0 改到 5.0

# 增加前进推力（更强的导航驱动）
'forward_thrust': 450.0          # 从 400.0 改到 450.0

# 增加航向控制增益（更快的转向响应）
'kp_yaw': 700.0                  # 从 600.0 改到 700.0
```

---

## 🔍 问题诊断表

### 问题 1：船不移动

```
症状：发送目标后，船没有反应
诊断步骤：
1. 检查节点是否都在运行
   ros2 node list | grep all_in_one

2. 检查是否收到目标
   ros2 topic echo /planning/goal

3. 检查推进器命令是否发出
   ros2 topic echo /wamv/thrusters/left/thrust

4. 检查日志错误
   ros2 launch control all_in_one_bringup.launch.py 2>&1 | grep ERROR
```

### 问题 2：避障不工作

```
症状：船不避开障碍物，直接撞上
诊断步骤：
1. 检查激光雷达数据
   ros2 topic echo /wamv/sensors/lidars/lidar_wamv_sensor/scan | head -20

2. 检查激光雷达频率
   ros2 topic hz /wamv/sensors/lidars/lidar_wamv_sensor/scan
   （应该 > 5 Hz）

3. 检查避障参数
   ros2 param get /all_in_one_stack obstacle_slow_dist
   ros2 param get /all_in_one_stack obstacle_stop_dist

4. 检查是否实际检测到障碍物
   # 向扫描数据中添加日志
```

### 问题 3：避障后不继续走（你的主要问题）

```
症状：船避开障碍物后停止或漂移，不继续朝向目标
诊断步骤：
1. 检查避障模式是否卡住
   ros2 launch control all_in_one_bringup.launch.py 2>&1 | grep "force_avoid"

2. 检查目标距离是否还在更新
   ros2 topic echo /planning/path | head -20

3. 检查目标航向是否正确
   # 应该每帧都更新目标方向

4. 可能的原因：
   a) full_clear_distance 太大（60.0） → 避障模式难以退出
   b) 目标坐标不正确 → 没有导航目标
   c) 推力不足 → 无法克服水阻
```

---

## 📊 实时监控仪表板

创建一个 shell 脚本来实时监控系统状态：

```bash
#!/bin/bash
# save as: monitor_system.sh

while true; do
    clear
    echo "=== UVAUTOBOAT SYSTEM MONITOR ==="
    echo ""
    
    # 节点状态
    echo ">>> NODES:"
    ros2 node list | grep -E "gps|pose|all_in_one" || echo "❌ Nodes not running"
    echo ""
    
    # 位置
    echo ">>> CURRENT POSITION:"
    ros2 topic echo -n 1 /wamv/pose_filtered 2>/dev/null | grep -E "position:|x:|y:" || echo "❌ No pose data"
    echo ""
    
    # 目标距离
    echo ">>> GOAL DISTANCE:"
    ros2 topic echo -n 1 /planning/path 2>/dev/null | head -5 || echo "❌ No path"
    echo ""
    
    # 推进器状态
    echo ">>> THRUSTERS:"
    echo -n "Left:  " && ros2 topic echo -n 1 /wamv/thrusters/left/thrust 2>/dev/null | grep data || echo "0"
    echo -n "Right: " && ros2 topic echo -n 1 /wamv/thrusters/right/thrust 2>/dev/null | grep data || echo "0"
    echo ""
    
    # 激光雷达
    echo ">>> LIDAR:"
    ros2 topic hz -c 1 /wamv/sensors/lidars/lidar_wamv_sensor/scan 2>/dev/null || echo "❌ No lidar"
    echo ""
    
    sleep 2
done
```

使用：
```bash
chmod +x monitor_system.sh
./monitor_system.sh
```

---

## ✅ 快速验证检查表

在运行完整测试前，快速检查：

```
□ ROS2 是否正确安装？
  ros2 --version

□ 工作空间是否编译成功？
  colcon build --packages-select control plan

□ 所有依赖是否安装？
  rosdep install --from-paths src --ignore-src -r -y

□ 是否在仿真环境中（Gazebo）？
  （决定了话题名称）

□ launch 文件语法是否正确？
  python3 -m py_compile control/launch/all_in_one_bringup.launch.py
```

---

## 🚀 下一步

**推荐测试顺序：**

1. **第一天：** 基础验证（第一阶段）
   - ✅ 节点都启动
   - ✅ 话题都活跃
   - ✅ 数据频率正常

2. **第二天：** 避障测试（第二阶段）
   - ✅ 船能朝向目标移动
   - ✅ 遇到障碍物能转向
   - ✅ 避开后能恢复导航

3. **第三天：** 坐标系验证（第三阶段）
   - ✅ 目标坐标计算正确
   - ✅ ENU 坐标系一致

4. **可选：** 参数微调（第四阶段）
   - 根据实际表现调整参数

---

## 📞 常见问题

**Q: 我应该把这些诊断命令放在哪里运行？**
A: 每个命令应该在单独的终端运行。建议打开 4-5 个终端：
- 终端1：启动 launch 文件
- 终端2：发送目标命令
- 终端3：监看日志
- 终端4-5：运行诊断命令

**Q: 如果激光雷达数据为 0，说明什么？**
A: 说明扫描数据中有零值（可能是发散的光线或噪声）。这是正常的，代码会过滤掉。

**Q: 我看不到日志输出，怎么办？**
A: 确保使用了 `output='screen'` 参数在 launch 文件中，或用：
```bash
ros2 launch control all_in_one_bringup.launch.py 2>&1 | tee launch.log
```

