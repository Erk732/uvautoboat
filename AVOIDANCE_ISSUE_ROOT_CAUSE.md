# 避障后不继续朝向目标的根本原因分析

## 🔴 核心问题代码分析

### 问题代码位置（第 887-895 行）

```python
# Force avoidance whenever any sector is below full_clear_distance; resume only when all clear
clear_val = self.full_clear_distance       # 默认值: 60.0 m
f_val = front_min if front_min is not None else clear_val
l_val = left_min if left_min is not None else clear_val
r_val = right_min if right_min is not None else clear_val

if f_val < clear_val or l_val < clear_val or r_val < clear_val:
    self.force_avoid_active = True        # ⚠️ 激活避障模式
elif f_val >= clear_val and l_val >= clear_val and r_val >= clear_val:
    self.force_avoid_active = False       # ⚠️ 只有所有方向都超过60m才退出
```

### ❌ 问题 1：避障模式卡住（最严重）

**场景：**
```
激光雷达感应范围通常是 ~30m，但 full_clear_distance 设置为 60m

激光数据：
- 前方：20m 有障碍物
- 左方：30m（扫描范围末端）
- 右方：30m（扫描范围末端）

代码逻辑判断：
f_val = 20 < 60 → force_avoid_active = True  ✅ 激活避障
l_val = 30 < 60 → force_avoid_active = True  ⚠️ 问题！
r_val = 30 < 60 → force_avoid_active = True  ⚠️ 问题！

即使船已经避开了障碍物（前方25m），因为激光扫描范围的边界
（30m）小于阈值（60m），避障模式永远无法退出！

退出条件需要：f_val >= 60 AND l_val >= 60 AND r_val >= 60
但激光雷达扫描范围只有 30-35m，不可能达到 60m！
```

**结果：** `force_avoid_active` 永久为 `True`，避障模式卡住！

---

### ❌ 问题 2：当卡在避障模式时发生什么？

查看第 935 行的逻辑：

```python
if self.force_avoid_active and front_min_eff >= self.obstacle_slow_dist:
    front_min_eff = self.obstacle_slow_dist - 0.01  # 人为降低前方距离
    
    # 当前方障碍物、航向误差大时，削弱前进推力
    if self.heading_align_thresh > 0.0 and abs(e_yaw) > self.heading_align_thresh:
        left_cmd *= 0.2    # ⚠️ 推力减弱到 20%！
        right_cmd *= 0.2   # ⚠️ 推力减弱到 20%！
```

**问题：** 当 `force_avoid_active=True` 且航向误差大时，推力被强制降低到 20%
- 这导致船即使没有障碍物，也会以极低速度蠕动
- 目标距离虽然在减小，但速度太慢，看起来"不走"

---

### ❌ 问题 3：极坐标直方图干扰目标导航

第 990-1005 行：

```python
if self.force_avoid_active or front_min_eff < self.obstacle_slow_dist:
    polar_bias = self._polar_bias_from_scan()  # 纯根据障碍物分布转向
    diff_bias += polar_bias * self.avoid_diff_gain

    # 如果左右都看不出差别，强制选一侧转
    if self.force_avoid_active and abs(diff_bias) < 1e-6:
        turn_dir_force = 1.0
        # ... 计算转向方向 ...
        diff_bias = 0.5 * self.avoid_diff_gain * turn_dir_force  # ⚠️ 强制转向
```

**问题：** 当 `force_avoid_active=True` 时，即使前方无障碍物，也会根据极坐标直方图强制转向
- 这会覆盖朝向目标的转向命令
- 导致船不能有效朝向目标前进

---

## 🎯 根本原因总结

```
当前系统有 3 个缺陷导致"避障后不继续走"：

1. full_clear_distance (60m) 太大
   └─ 激光雷达范围通常 30-35m
   └─ 避障模式永久卡住（force_avoid_active 无法关闭）

2. force_avoid_active 时强制削弱推力到 20%
   └─ 即使无障碍物，推力也很弱
   └─ 船蠕动，看起来不移动

3. 避障模式下用极坐标直方图强制转向
   └─ 覆盖朝向目标的转向命令
   └─ 船没有朝目标方向，而是根据障碍物分布转向
```

---

## ✅ 解决方案

### 方案 A：降低 full_clear_distance（最简单 ⭐⭐⭐⭐⭐）

**修改参数（launch 文件）：**

```python
# 当前值（导致问题）
'full_clear_distance': 60.0,

# 改为：
'full_clear_distance': 20.0,  # 与激光范围匹配
```

**原理：**
- 激光雷达范围 ~30m，所以阈值应该 < 30m
- 20m 是合理的，表示"前后左右都 20m 以上无障碍"
- 这样避障模式可以正常退出

**效果：**
```
前方：25m（无障碍） ✅
左方：30m          ✅
右方：30m          ✅

判断：25 >= 20, 30 >= 20, 30 >= 20
结果：force_avoid_active = False ✅ 正常退出避障！
```

**实施时间：** 5 分钟（改 1 个参数，重启）

---

### 方案 B：改进避障退出逻辑（更精确）

在代码中修改第 887-895 行：

**当前代码（有问题）：**
```python
clear_val = self.full_clear_distance
f_val = front_min if front_min is not None else clear_val
l_val = left_min if left_min is not None else clear_val
r_val = right_min if right_min is not None else clear_val
if f_val < clear_val or l_val < clear_val or r_val < clear_val:
    self.force_avoid_active = True
elif f_val >= clear_val and l_val >= clear_val and r_val >= clear_val:
    self.force_avoid_active = False
```

**改为（更聪明）：**
```python
clear_val = self.full_clear_distance

# 只看有效的传感器值（不要用默认值）
valid_distances = []
if front_min is not None:
    valid_distances.append(('front', front_min))
if left_min is not None:
    valid_distances.append(('left', left_min))
if right_min is not None:
    valid_distances.append(('right', right_min))

# 激活：只要任何方向 < 阈值
should_avoid = any(dist < clear_val for _, dist in valid_distances)

# 退出：所有有效方向都 > 阈值
should_exit = all(dist >= clear_val * 0.8 for _, dist in valid_distances) if valid_distances else True

if should_avoid:
    self.force_avoid_active = True
elif should_exit:
    self.force_avoid_active = False
```

**优势：**
- 不依赖激光扫描范围的边界值
- 更灵活的退出逻辑（80% 阈值可调）
- 避免模式卡住

**实施难度：** ⭐⭐⭐（需要改代码，但逻辑简单）

---

### 方案 C：分离避障和导航目标（最优 ⭐⭐⭐⭐⭐）

**关键改进：** 当避障时，保持朝向目标的意图

**修改第 1005 行附近：**

```python
# 当前代码（纯避障，忽视目标）
if self.force_avoid_active or front_min_eff < self.obstacle_slow_dist:
    polar_bias = self._polar_bias_from_scan()
    diff_bias += polar_bias * self.avoid_diff_gain

# 改为（混合避障 + 目标）
if self.force_avoid_active or front_min_eff < self.obstacle_slow_dist:
    polar_bias = self._polar_bias_from_scan()
    
    # 70% 权重给目标导航，30% 权重给避障
    target_bias = e_yaw / max(1.0, abs(e_yaw) or 1.0) * self.avoid_diff_gain * 0.7
    avoid_bias = polar_bias * self.avoid_diff_gain * 0.3
    
    diff_bias += target_bias + avoid_bias
else:
    diff_bias += polar_bias * self.avoid_diff_gain
```

**效果：**
- 避障时仍然部分保持朝向目标
- 避开障碍物，同时向目标靠近
- 避障后顺利恢复导航

**实施难度：** ⭐⭐⭐⭐（需要理解权重分配）

---

## 🚀 立即行动建议

### 第 1 步：快速修复（5 分钟）

编辑 `control/launch/all_in_one_bringup.launch.py`，修改：

```python
# 找到这一行：
'full_clear_distance': 60.0,

# 改为：
'full_clear_distance': 20.0,
```

重新启动系统，测试是否解决问题。

**预期效果：** 避障模式能正常退出，船能继续朝向目标。

---

### 第 2 步：如果第 1 步还有问题（10 分钟）

实施方案 C：修改 `all_in_one_stack.py` 第 990-1005 行，使用混合避障+导航的策略。

---

### 第 3 步：精细调优（可选）

根据实际测试结果，调整：
- `obstacle_slow_dist`: 激活软避障的距离
- `obstacle_stop_dist`: 激活硬避障的距离
- 混合权重（目标 vs 避障）

---

## 📊 参数对比

| 参数 | 当前值 | 推荐值 | 说明 |
|------|--------|--------|------|
| `full_clear_distance` | 60.0 ❌ | 20.0 ✅ | 导致模式卡住 |
| `obstacle_slow_dist` | 15.0 | 12.0 | 激活软避障的距离 |
| `obstacle_stop_dist` | 8.0 | 6.0 | 激活硬避障的距离 |
| `avoid_diff_gain` | 40.0 | 40.0 | 避障转向增益（可保持） |

---

## 总结

**问题源头：** `full_clear_distance: 60.0` 太大，激光雷达无法探测到 60m，导致避障模式永久激活。

**最快解决：** 将 `full_clear_distance` 改为 20.0。

**完美解决：** 混合避障策略（方案 C），使避障与导航目标并行工作。

你想先试第 1 步（改参数）还是直接实施完整方案？

