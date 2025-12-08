# 避障代码视觉化指南

## 🎯 避障决策树

```
                    ┌─────────────────────┐
                    │  获取激光数据        │
                    │ analyze_lidar()     │
                    └──────────┬──────────┘
                               │
                   ┌───────────▼──────────────┐
                   │ front_min, left_min,     │
                   │ right_min = ?            │
                   └───────────┬──────────────┘
                               │
              ┌────────────────▼────────────────┐
              │ 对比 full_clear_distance        │
              │ (现在改成了 20.0)              │
              └────────┬───────────────────────┘
                       │
         ┌─────────────▼──────────────┐
         │ 全部 > 20m?                │
         └─────────────┬──────────────┘
                 ┌─────┴─────┐
               否│            │是
         ┌──────▼────┐   ┌───▼─────┐
    YES │激活避障    │   │退出避障  │
         │force_     │   │force_    │
         │avoid_     │   │avoid_    │
         │active=T   │   │active=F  │
         └──────┬────┘   └───┬──────┘
                │            │
         ┌──────▼────────────▼──┐
         │ 检查是否有硬障碍     │
         │ (front < 6.0m?)      │
         └──────┬──────┬────────┘
                │      │
            是  │      │  否
         ┌──────▼┐   ┌─▼─────────┐
         │硬避障 │   │检查软障碍  │
         │模式   │   │(f<12m?)   │
         └──┬────┘   └──┬────────┘
            │           │
         ┌──▼───┐    ┌──▼──┐
         │ 反向  │    │软转 │
         │+转向  │    │+降速│
         └───┬──┘    └──┬──┘
             │          │
          ┌──▼──────────▼──┐
          │发送推进器命令   │
          │publish_thrust  │
          └────────────────┘
```

---

## 📍 代码位置速查

### 在哪些行数可以找到相关代码？

```
避障初始化与参数:
├─ 第 110-145 行  ← 参数声明（obstacle_slow_dist 等）
└─ 第 163-195 行  ← 参数读取

激光传感器处理:
├─ 第 508-555 行  ← analyze_lidar() [核心！获取障碍物距离]
├─ 第 556-639 行  ← _analyze_pointcloud() [3D点云处理]
├─ 第 641-668 行  ← _polar_bias_from_scan() [极坐标转向]
└─ 第 670-729 行  ← _vfh_steer() [VFH向量场转向]

主避障控制（最重要）:
├─ 第 887-895 行  ← ❌❌❌ 问题所在！全局避障激活/退出逻辑
├─ 第 908-925 行  ← 硬避障状态机（反向+转向）
├─ 第 934-1010 行 ← 软避障（降速+转向）
└─ 第 1013 行     ← 避障模式复位

推进器输出:
└─ 第 1019-1025 行 ← publish_thrust() [发送命令]
```

---

## 🔴 问题所在位置（第 887-895 行）

```python
# ❌❌❌ 这就是导致避障卡住的代码！

clear_val = self.full_clear_distance       # 原值: 60.0
f_val = front_min if front_min is not None else clear_val
l_val = left_min if left_min is not None else clear_val
r_val = right_min if right_min is not None else clear_val

if f_val < clear_val or l_val < clear_val or r_val < clear_val:
    self.force_avoid_active = True      # ❌ 激活

elif f_val >= clear_val and l_val >= clear_val and r_val >= clear_val:
    self.force_avoid_active = False     # ❌ 退出条件太苛刻
```

**为什么卡住：**
```
激光最大范围: 30m
要求退出的距离: 60m > 30m
结果: 永远无法退出
```

**修复后：**
```python
'full_clear_distance': 20.0     # ✅ 改成 20.0
```

---

## 🟢 三个避障阶段详解

### 1️⃣ 硬避障（obstacle < 6m）

```
当 front_min < 6.0m 时触发

行为:
  阶段1: 反向 (3秒)     avoid_mode = 'reverse'
    left_thrust  = -200.0
    right_thrust = -200.0
    效果: 船后退

  阶段2: 转向 (3秒)     avoid_mode = 'turn'
    left_thrust  = -350 * turn_dir
    right_thrust = +350 * turn_dir
    效果: 原地旋转，等待前方清空

检查: 
  - front_min > 11m (8 + 3) 或
  - 转向超过 5 秒
  → 退出硬避障
```

### 2️⃣ 软避障（6m < obstacle < 12m）

```
当 6 < front_min < 12m 时触发

计算推力缩放:
  scale = (front_min - 6) / (12 - 6)
  scale ∈ [0, 1]
  
  但被限制为 [0.2, 1.0]
  最多降到 20% 推力

转向增益来自三个地方:
  1. 左右距离差 (diff_bias)
  2. VFH 最安全方向 (vfh_angle)
  3. 极坐标直方图 (polar_bias) ← 可能干扰目标导航

示例:
  前方 9m，左方 15m，右方 5m
  → scale = (9-6)/(12-6) = 0.5
  → left_cmd *= 0.5
  → right_cmd *= 0.5
  → diff_bias = (5-15)/15 * 40 = -26.7
  → left_cmd -= (-26.7) = left_cmd += 26.7 (增强左推)
  → right_cmd += (-26.7) = right_cmd -= 26.7 (减弱右推)
  → 结果: 向右转，并降速
```

### 3️⃣ 正常导航（obstacle > 12m）

```
当 front_min > 12m 时

推力计算:
  T_forward = 400.0
  T_diff = kp_yaw * e_yaw
  
  left_cmd = T_forward - T_diff
  right_cmd = T_forward + T_diff

例如:
  航向偏差 e_yaw = 0.1 rad
  T_diff = 600 * 0.1 = 60
  left_cmd = 400 - 60 = 340
  right_cmd = 400 + 60 = 460
  
  结果: 左推减速 340，右推加速 460
       → 船向右转，朝向目标
```

---

## 📊 激光扫描示意图

```
俯视图 (船向上看):

                    前方 30°
                  /    ↑    \
                /      |      \
              /        |        \
            /          |          \
          /    [目标]  |  [激光]    \
          \            |            /
            \          |          /
              \        |        /
                \      |      /
                  \    ↓    /
                    后方 30°

角度说明:
  0°   = 船的右前方
  90°  = 船的左侧
  180° = 船的后方
  -90° = 船的右侧

激光扫描范围:
  通常 -π 到 +π (-180° 到 +180°)
  即 360° 完整覆盖

前方范围 (front):
  -30° 到 +30° (共 60°)
  
左方范围 (left):
  0° 到 +60°
  
右方范围 (right):
  -60° 到 0°
```

---

## 🧠 避障"思维过程"

```
时刻 T (200ms 间隔):

T0: 
  收到目标: (100, 0)
  生成路径
  激活: active = True

T1 (50ms):
  current_pose = (-0.975, 25.282)
  analyze_lidar():
    front_min = 25.0m
    left_min = 30.0m
    right_min = 30.0m
  
  判断: 25 < 60? YES
  → force_avoid_active = True ❌
  
  计算转向:
    前方无障碍，无需转向
  
  发送推力:
    left_cmd = 400
    right_cmd = 400

T2 (100ms):
  analyze_lidar():
    front_min = 26.0m (无变化)
    left_min = 30.0m
    right_min = 30.0m
  
  判断: 26 < 60? YES
       30 < 60? YES
       30 < 60? YES
  → force_avoid_active = True  ❌ 仍在激活！
  
  因为激活了避障，推力被削弱:
    left_cmd *= 0.2 = 80
    right_cmd *= 0.2 = 80
  
  发送推力:
    left_cmd = 80 (❌ 很弱！)
    right_cmd = 80 (❌ 很弱！)

T3 (150ms):
  还是一样...force_avoid_active 还是 True
  推力还是 80 左右
  船蠕动...

用户感受: "船停止了" ❌
实际上: 船在以 20% 推力蠕动，避障模式永不退出
```

---

## ✅ 修复后的流程

```
改参数后: full_clear_distance = 20.0

T0:
  [同上]

T1:
  analyze_lidar():
    front_min = 25.0m
    left_min = 30.0m  
    right_min = 30.0m
  
  判断: 25 >= 20? YES
       30 >= 20? YES
       30 >= 20? YES
  → force_avoid_active = False ✅
  
  计算转向:
    e_yaw = 0.05 rad
    T_diff = 600 * 0.05 = 30
  
  发送推力:
    left_cmd = 400 - 30 = 370
    right_cmd = 400 + 30 = 430
  
用户感受: "船朝向目标走了！" ✅
```

---

## 🔑 关键代码片段

### 激活避障 (第 893 行)

```python
if f_val < clear_val or l_val < clear_val or r_val < clear_val:
    self.force_avoid_active = True

# 含义: 只要任意方向 < 阈值，就激活
# 例如: 前方 5m，侧方 60m → 激活 (因为 5 < 60)
```

### 退出避障 (第 895 行)

```python
elif f_val >= clear_val and l_val >= clear_val and r_val >= clear_val:
    self.force_avoid_active = False

# 含义: 必须所有方向都 >= 阈值，才退出
# 例如: 前方 60m，侧方 60m → 退出 (所有都 >= 60)
#      但激光只能看 30m！所以永不退出
```

### 软避障推力削弱 (第 943 行)

```python
left_cmd *= scale
right_cmd *= scale

# scale ∈ [0.2, 1.0]
# 当 scale = 0.2 时，推力下降到 20%！
# 这就是为什么 force_avoid_active=True 时船看起来"不走"
```

### 避障转向 (第 991-1005 行)

```python
if self.force_avoid_active or front_min_eff < self.obstacle_slow_dist:
    polar_bias = self._polar_bias_from_scan()
    diff_bias += polar_bias * self.avoid_diff_gain

# 问题：force_avoid_active=True 时，会用极坐标直方图
# 这覆盖了朝向目标的转向，导致船不向目标走
```

---

## 📌 总结：三个关键问题点

| 问题 | 所在行 | 现象 | 修复 |
|------|--------|------|------|
| **卡在避障模式** | 887-895 | 推力锁定20% | 改 full_clear_distance: 60→20 |
| **推力削弱** | 943 | 船蠕动不走 | 同上 (模式退出，推力恢复) |
| **忽视目标方向** | 991-1005 | 转向不是朝目标 | 改权重或改代码(可选) |

最快修复: 改一个参数 (60.0 → 20.0)
彻底修复: 改代码逻辑 (混合避障和导航)

