# all_in_one_stack.py 避障部分详细讲解

## 📍 代码位置汇总

```
函数声明位置:
├─ analyze_lidar()             第 508 行  ← 核心！获取障碍物距离
├─ _analyze_pointcloud()       第 556 行  ← 处理3D点云数据
├─ _polar_bias_from_scan()     第 641 行  ← 极坐标直方图转向
├─ _vfh_steer()                第 670 行  ← VFH向量场转向
└─ control_loop()              第 741 行  ← 主控制循环（包含避障逻辑）

关键参数: 第 110-140 行
关键状态: 第 224-237 行
```

---

## 🔍 第 1 部分：参数声明与初始化（第 60-145 行）

### 避障相关参数

```python
# 第 123-137 行 - 避障参数定义

self.declare_parameter('obstacle_slow_dist', 15.0)     # 激活软避障的距离 (m)
self.declare_parameter('obstacle_stop_dist', 8.0)      # 激活硬避障的距离 (m)
self.declare_parameter('avoid_turn_thrust', 350.0)     # 硬避障时的转向推力 (N)
self.declare_parameter('avoid_diff_gain', 40.0)        # 左右转向增益
self.declare_parameter('avoid_clear_margin', 3.0)      # 退出避障的安全裕度 (m)
self.declare_parameter('avoid_max_turn_time', 5.0)     # 硬避障最长持续时间 (s)
self.declare_parameter('full_clear_distance', 60.0)    # ❌ 关键问题！全局清空距离

# 第 140 行 - 激光雷达参数
self.declare_parameter('front_angle_deg', 30.0)        # 前方扫描角度 (deg)
self.declare_parameter('side_angle_deg', 60.0)         # 侧方扫描角度 (deg)
self.declare_parameter('min_range_filter', 1.5)        # 最小距离滤波 (m)
```

### 参数含义解释

```
obstacle_slow_dist: 15.0
  含义: 当前方障碍物 < 15m 时，触发"软避障"
  效果: 降低推力到 20-100%，同时转向

obstacle_stop_dist: 8.0
  含义: 当前方障碍物 < 8m 时，触发"硬避障"
  效果: 原地旋转（推力差异很大），等待障碍物远离

avoid_turn_thrust: 350.0
  含义: 硬避障时的转向推力大小
  用途: 计算 left_cmd 和 right_cmd 的差值

avoid_diff_gain: 40.0
  含义: 根据左右激光距离差计算的转向增益
  范围: [-40, +40]
  规则: 左边远 → 向左转 (增益为正)
       右边远 → 向右转 (增益为负)

full_clear_distance: 60.0 ❌❌❌
  含义: 当前后左右距离都 > 60m 时，才退出避障模式
  问题: 激光范围只有 ~30m，无法达到 60m！
  结果: 避障模式永久激活！
```

---

## 🔍 第 2 部分：激光雷达分析（第 508-639 行）

### 核心函数：analyze_lidar()

```python
def analyze_lidar(self):
    """
    返回 (front_min, left_min, right_min) 三个距离值
    
    前方 (front):   从 -front_angle 到 +front_angle
                   默认: -30° 到 +30° (共60°)
    
    左方 (left):    从 0° 到 +side_angle  
                   默认: 0° 到 +60°
    
    右方 (right):   从 -side_angle 到 0°
                   默认: -60° 到 0°
    
    示意图（俯视图）：
                前方 30°
              /  ↑  \
            /    |    \
          /      |      \
        左60°    船    右60°
          \      |      /
            \    |    /
              \  ↓  /
              
    """
    
    # 第 516 行：首先尝试从3D点云获取
    front_min_cloud, left_min_cloud, right_min_cloud = self._analyze_pointcloud()
    
    # 第 519 行：然后从2D激光扫描获取
    if self.latest_scan is not None:
        scan = self.latest_scan
        angle = scan.angle_min          # 起始角度 (通常 -π)
        f = l = r = float('inf')         # 初始化为无穷大
        
        for rng in scan.ranges:          # 遍历每条激光射线
            # 第 525 行：过滤无效数据
            if math.isinf(rng) or math.isnan(rng) or rng <= 0.0:
                angle += scan.angle_increment
                continue
            
            # 第 528 行：过滤太近的数据（可能是自己）
            if rng < self.min_range_filter:  # min_range_filter = 1.5m
                angle += scan.angle_increment
                continue
            
            # 第 531-539 行：按角度分类（前/左/右）
            if -self.front_angle <= angle <= self.front_angle:
                f = min(f, rng)          # 记录前方最小距离
            
            if 0.0 <= angle <= self.side_angle:
                l = min(l, rng)          # 记录左方最小距离
            
            if -self.side_angle <= angle <= 0.0:
                r = min(r, rng)          # 记录右方最小距离
            
            angle += scan.angle_increment
    
    # 第 546 行：合并点云和扫描数据
    # 优先使用点云（更准确），否则用扫描
    front_min = front_min_cloud or front_min_scan
    left_min = left_min_cloud or left_min_scan
    right_min = right_min_cloud or right_min_scan
    
    return front_min, left_min, right_min
```

### 子函数：_analyze_pointcloud()

```python
def _analyze_pointcloud(self):
    """
    处理3D点云数据（PointCloud2格式）
    
    为什么需要3D点云？
    └─ LaserScan 是2D的（只有range和angle）
    └─ 点云有Z坐标，可以检测低于水面的障碍物（如码头桩）
    
    """
    
    if self.latest_cloud is None:
        return None, None, None
    
    # 第 568 行：读取点云中的每个点
    for p in point_cloud2.read_points(self.latest_cloud, 
                                      field_names=('x', 'y', 'z')):
        x, y, z = p
        
        # 第 571 行：Z轴过滤（你之前改的！）
        if z < self.cloud_z_min or z > self.cloud_z_max:
            # cloud_z_min = -10.0 (水面以下)
            # cloud_z_max = 3.0   (水面以上)
            # 只保留这个高度范围内的点
            continue
        
        # 第 574 行：计算水平距离
        dist = math.hypot(x, y)          # √(x² + y²)
        
        # 第 575-576 行：距离过滤
        if dist <= 0.0 or dist < self.min_range_filter:
            continue
        
        # 第 577 行：计算角度
        angle = math.atan2(y, x)        # 从X轴正方向逆时针测量
        
        # 第 579-589 行：按角度分类（同 analyze_lidar）
        if -self.front_angle <= angle <= self.front_angle:
            front_min = dist if front_min is None else min(front_min, dist)
        
        if 0.0 <= angle <= self.side_angle:
            left_min = dist if left_min is None else min(left_min, dist)
        
        if -self.side_angle <= angle <= 0.0:
            right_min = dist if right_min is None else min(right_min, dist)
    
    return front_min, left_min, right_min
```

---

## 🔍 第 3 部分：转向算法（第 641-720 行）

### 子函数：_polar_bias_from_scan()（极坐标直方图）

```python
def _polar_bias_from_scan(self):
    """
    简单的极坐标直方图：比较左右两侧的"自由空间"
    
    原理：
    └─ 遍历激光扫描的所有射线
    └─ 左侧射线（angle > 0）贡献到 left_score
    └─ 右侧射线（angle < 0）贡献到 right_score
    └─ 权重为 range^power（距离越远，权重越高）
    └─ 最后计算偏置: (left - right) / total
    
    返回值范围: [-1, 1]
    └─ +1:  强烈转左（左边很宽敞）
    └─ 0:   没有偏好（左右一样宽敞）
    └─ -1:  强烈转右（右边很宽敞）
    """
    
    if not self.polar_use_scan or self.latest_scan is None:
        return 0.0
    
    scan = self.latest_scan
    angle = scan.angle_min              # 起始角度 -π
    step = scan.angle_increment         # 角度增量 ~0.006 rad
    left_score = 0.0
    right_score = 0.0
    power = max(self.polar_weight_power, 0.0)  # 通常 1.0
    
    for r in scan.ranges:               # 遍历每条射线
        # 第 656 行：距离过滤
        if r < self.polar_min_range:    # 最小 0.5m
            r = self.polar_min_range
        
        # 第 657 行：计算权重（距离的幂次）
        w = r ** power                  # 通常 r^1 = r
        
        # 第 658-661 行：左右分类累加
        if angle > 0.0:
            left_score += w             # 左侧累加
        else:
            right_score += w            # 右侧累加
        
        angle += step
    
    # 第 662 行：归一化
    total = left_score + right_score
    if total <= 0.0:
        return 0.0
    
    bias = (left_score - right_score) / total
    # 结果在 [-1, 1] 范围内
    return bias
```

### 子函数：_vfh_steer()（向量场直方图）

```python
def _vfh_steer(self, desired_yaw: float):
    """
    向量场直方图（VFH）算法：找到最接近目标航向的"自由"方向
    
    工作步骤：
    1. 将360°扫描分成多个"bin"（通常5°一个）
    2. 标记那些有障碍物的bin（距离 < vfh_block_dist）
    3. 标记bin周围的区域为"被挡住"（膨胀处理）
    4. 在所有自由bin中，选择最接近目标航向的
    
    """
    
    if not self.vfh_enabled or self.latest_scan is None:
        return None
    
    scan = self.latest_scan
    bin_rad = math.radians(max(self.vfh_bin_deg, 1e-3))  # 5° = 0.087 rad
    num_bins = int(math.ceil((scan.angle_max - scan.angle_min) / bin_rad))
    blocked = [False] * num_bins
    
    # 第 691-702 行：标记被挡住的bin
    angle = scan.angle_min
    step = scan.angle_increment
    for r in scan.ranges:
        idx = int((angle - scan.angle_min) / bin_rad)
        if 0 <= idx < num_bins:
            if r > 0.0 and r < self.vfh_block_dist:  # vfh_block_dist = 10.0m
                blocked[idx] = True
        angle += step
    
    # 第 703-714 行：膨胀被挡住的bin（添加安全裕度）
    clearance = math.radians(self.vfh_clearance_deg)
    inflate_bins = int(math.ceil(clearance / bin_rad))
    if inflate_bins > 0:
        blocked_inf = blocked[:]
        for i, b in enumerate(blocked):
            if not b:
                continue
            # 标记周围的bin也被挡住
            for k in range(-inflate_bins, inflate_bins + 1):
                j = i + k
                if 0 <= j < num_bins:
                    blocked_inf[j] = True
        blocked = blocked_inf
    
    # 第 715-729 行：找最接近目标航向的自由bin
    desired_idx = int((desired_yaw - scan.angle_min) / bin_rad)
    best_idx = None
    best_err = None
    
    for i, b in enumerate(blocked):
        if b:                          # 跳过被挡住的
            continue
        center_ang = scan.angle_min + (i + 0.5) * bin_rad
        err = abs(math.atan2(math.sin(center_ang - desired_yaw), 
                             math.cos(center_ang - desired_yaw)))
        if best_err is None or err < best_err:
            best_err = err
            best_idx = i
    
    if best_idx is None:
        return None
    
    return scan.angle_min + (best_idx + 0.5) * bin_rad
```

---

## 🔍 第 4 部分：主避障控制逻辑（第 887-1015 行）

这是最关键的部分！让我详细讲解避障的三个阶段：

### 阶段 1：全局避障模式激活/退出（第 887-895 行）

```python
# ========== 关键代码！ ==========

# Force avoidance whenever any sector is below full_clear_distance; 
# resume only when all clear
clear_val = self.full_clear_distance       # 默认 60.0 ❌❌❌

f_val = front_min if front_min is not None else clear_val
l_val = left_min if left_min is not None else clear_val
r_val = right_min if right_min is not None else clear_val

if f_val < clear_val or l_val < clear_val or r_val < clear_val:
    self.force_avoid_active = True      # 激活全局避障模式
elif f_val >= clear_val and l_val >= clear_val and r_val >= clear_val:
    self.force_avoid_active = False     # 退出全局避障模式
```

**问题分析：**

```python
# 典型的激光数据：
front_min = 25.0    # 前方 25m（无障碍）
left_min = 30.0     # 左方 30m（扫描范围末端）
right_min = 30.0    # 右方 30m（扫描范围末端）

# 判断 (clear_val = 60.0)：
f_val = 25.0 < 60.0 → True     ❌ 激活！
l_val = 30.0 < 60.0 → True     ❌ 保持激活！
r_val = 30.0 < 60.0 → True     ❌ 保持激活！

结果：force_avoid_active = True  （永久卡住！）

要退出需要：25 >= 60 AND 30 >= 60 AND 30 >= 60
这永远无法满足（激光最远只能看 30m）
```

### 阶段 2：硬避障状态机（第 908-925 行）

```python
# 硬避障的两个阶段：反向 + 转向

if self.avoid_mode in ('reverse', 'turn'):
    if self.avoid_mode == 'reverse':
        # 阶段1：反向 (持续 recover_reverse_time = 3.0 秒)
        if (now_s - self.avoid_start_time) < self.recover_reverse_time:
            self.publish_thrust(self.recover_reverse_thrust,    # -200.0
                               self.recover_reverse_thrust)    # -200.0
            # 推力都是负的，船向后退
            return
        
        # 3秒后切换到转向
        self.avoid_mode = 'turn'
        self.avoid_start_time = now_s
    
    if self.avoid_mode == 'turn':
        # 阶段2：转向 (基于左右障碍物选择方向)
        clear_dist = self.obstacle_stop_dist + self.avoid_clear_margin
        time_in_turn = now_s - self.avoid_start_time
        
        if (front_min is None or front_min > clear_dist) or (time_in_turn > self.avoid_max_turn_time):
            # 前方无障碍或转向超时，结束硬避障
            self.avoid_mode = ''
            self.avoid_start_time = 0.0
        else:
            # 继续转向
            turn_cmd = self.avoid_turn_thrust * self.avoid_turn_dir
            self.publish_thrust(-turn_cmd, turn_cmd)  # 左推进器推、右推进器拉（或反之）
            return
```

### 阶段 3：软避障（第 934-1010 行）

```python
# 当障碍物在 obstacle_slow_dist (12m) 到 obstacle_stop_dist (6m) 之间时

if front_min_eff < self.obstacle_slow_dist:  # 12.0
    # 软避障：降低推力 + 转向
    
    denom = max(self.obstacle_slow_dist - self.obstacle_stop_dist, 0.1)
    # denom = 12 - 6 = 6
    
    scale = (front_min_eff - self.obstacle_stop_dist) / denom
    # 当 front_min = 6m 时，scale = 0 / 6 = 0
    # 当 front_min = 12m 时，scale = 6 / 6 = 1
    
    scale = max(0.2, min(1.0, scale))
    # 限制在 [0.2, 1.0]，最多降到 20% 推力
    
    left_cmd *= scale       # ❌ 推力被削弱
    right_cmd *= scale      # ❌ 推力被削弱
    
    # ===================== 关键：转向计算 =====================
    
    # 方式1：根据左右距离差转向
    diff_bias = (right_min_eff - left_min_eff) / norm * self.avoid_diff_gain
    # 左边远 → right_min > left_min → diff_bias > 0 → 向左转
    # 右边远 → right_min < left_min → diff_bias < 0 → 向右转
    
    # 方式2：VFH 转向（选择最安全的方向）
    vfh_angle = self._vfh_steer(desired_yaw)
    if vfh_angle is not None:
        rel = normalize_angle(vfh_angle)
        diff_bias += max(-1.0, min(1.0, rel / max(self.front_angle, 1e-3))) * self.avoid_diff_gain
    
    # 方式3：极坐标直方图（比较左右自由空间）
    if self.force_avoid_active or front_min_eff < self.obstacle_slow_dist:
        polar_bias = self._polar_bias_from_scan()
        # polar_bias ∈ [-1, 1]
        # +1: 左边宽敞
        # -1: 右边宽敞
        diff_bias += polar_bias * self.avoid_diff_gain
    
    # ❌ 问题：当 force_avoid_active=True 时
    # polar_bias 会覆盖朝向目标的转向意图！
    
    # 最后：应用转向偏置
    left_cmd -= diff_bias       # 向左转时，左推进减弱
    right_cmd += diff_bias      # 向左转时，右推进增强
```

---

## 🚨 避障卡住的根本流程

```
时间序列：

T=0s: 收到目标 → 路径生成 → 开始导航

T=1s: 前方无障碍
     front_min = 25m > 60m? NO
     force_avoid_active = TRUE  ❌ 激活

T=2s: 前方仍无障碍，但还在激活
     left_min = 30m > 60m? NO
     right_min = 30m > 60m? NO
     force_avoid_active = TRUE  ❌ 卡住！

T=3-10s: 持续卡住
     推力被削弱到 20%
     left_cmd *= 0.2
     right_cmd *= 0.2
     ↓
     船蠕动，看起来不走

用户看到：避障后船停止 ❌

实际上：推力卡在 20%，一直尝试避障，导致速度极慢
```

---

## ✅ 修复的关键

改一个数字解决问题：

```python
# 改前：
'full_clear_distance': 60.0      # ❌ 激光无法探测 60m

# 改后：
'full_clear_distance': 20.0      # ✅ 激光能探测 20m

原因：
  激光雷达最大范围: ~30m
  新阈值: 20m < 30m
  结果: 可以正常退出避障模式！
```

---

## 📊 完整的控制流程图

```
┌─────────────────────────────────────┐
│  control_loop() 每 50ms 执行一次     │
└──────────────┬──────────────────────┘
               │
        ┌──────▼──────┐
        │ 获取激光数据  │
        └──────┬──────┘
               │
        ┌──────▼────────────────────────┐
        │ analyze_lidar()                │
        │ (返回 front_min, left_min,     │
        │  right_min)                    │
        └──────┬────────────────────────┘
               │
    ┌──────────▼──────────────┐
    │ 检查 force_avoid_active  │
    └──────────┬──────────────┘
               │
        ┌──────▼──────────────────┐
        │ 是否 < full_clear_distance? │
        └──────┬──────────────────┘
               │
        ┌──────▼──────────────────┐
        │ 激活/退出避障模式       │
        └──────┬──────────────────┘
               │
        ┌──────▼──────────────────┐
        │ 根据避障阶段控制推力    │
        │ 硬避障: 转向+反向       │
        │ 软避障: 降速+转向       │
        │ 正常:  全速+朝向目标    │
        └──────┬──────────────────┘
               │
        ┌──────▼──────────────────┐
        │ publish_thrust()         │
        │ (发送左右推进器命令)    │
        └──────────────────────────┘
```

---

## 💡 关键参数速查表

| 参数 | 值 | 含义 |
|------|-----|------|
| `forward_thrust` | 400.0 | 基础前进推力 |
| `kp_yaw` | 600.0 | 航向控制增益 |
| `obstacle_slow_dist` | 12.0 | 软避障激活距离 |
| `obstacle_stop_dist` | 6.0 | 硬避障激活距离 |
| `avoid_turn_thrust` | 350.0 | 硬避障转向推力 |
| `avoid_diff_gain` | 40.0 | 避障转向增益 |
| `full_clear_distance` | 20.0 ✅ | 全局避障退出距离 |
| `front_angle_deg` | 30.0 | 前方扫描角 |
| `side_angle_deg` | 60.0 | 侧方扫描角 |
| `cloud_z_min` | -10.0 | 点云Z下限 |
| `cloud_z_max` | 3.0 | 点云Z上限 |
| `min_range_filter` | 3.0 | 最小距离过滤 |

---

## 🎯 总结

避障系统由三个部分组成：

1. **传感器数据处理** (analyze_lidar)
   - 从激光数据提取前/左/右三个方向的最小距离

2. **转向决策** (VFH + 极坐标直方图)
   - 选择最安全或最接近目标的方向

3. **推力控制** (control_loop)
   - 根据避障阶段调整左右推进器推力

问题出在第3部分：`full_clear_distance: 60.0` 太大，导致避障模式卡住。
修复：改成 20.0，匹配激光雷达的实际范围。

