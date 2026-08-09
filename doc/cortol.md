# 四足机器狗曲线路径平滑跟踪方案

## 1. 目标

四足机器狗通常支持三个平面速度控制量：

```text
vx：机体前向速度
vy：机体横向速度
wz：绕 Z 轴角速度
```

虽然机器狗具备横移能力，但在正常路径跟踪过程中，希望它尽量像小车一样运动：

```text
主要使用：vx + wz
尽量不用：vy
```

即：

```text
通过转动机身方向来贴合路径，
而不是通过横向平移来纠正路径误差。
```

正常情况下设置：

```text
vy = 0
```

只有在严重偏离路径、狭窄通道恢复或局部避障时，才允许少量横移。

---

## 2. 机器人运动模型

将四足机器狗近似看作一个非完整约束移动机器人。

机器人状态为：

```text
x：世界坐标系下的 X 位置
y：世界坐标系下的 Y 位置
yaw：机器人当前朝向
```

控制量为：

```text
vx：机器人机体前向速度
wz：机器人角速度
```

运动学模型：

```math
\dot{x} = v_x \cos(\theta)
```

```math
\dot{y} = v_x \sin(\theta)
```

```math
\dot{\theta} = \omega
```

正常路径跟踪时：

```math
v_y = 0
```

ROS2 速度指令示例：

```cpp
geometry_msgs::msg::Twist cmd_vel;

cmd_vel.linear.x  = vx;
cmd_vel.linear.y  = 0.0;
cmd_vel.angular.z = wz;
```

---

## 3. 整体控制流程

推荐的路径跟踪流程如下：

```text
全局路径
   │
   ▼
获取机器狗当前位置和姿态
   │
   ▼
搜索距离机器狗最近的路径点
   │
   ▼
沿路径向前搜索前视点
   │
   ▼
计算路径方向、横向误差和航向误差
   │
   ▼
计算前向速度 vx 和角速度 wz
   │
   ▼
曲率限速
   │
   ▼
加速度和角加速度限制
   │
   ▼
发送 vx、vy=0、wz
```

---

## 4. 不要按照时间追踪路径点

错误方式：

```text
运行时间增加
   │
   ▼
路径索引不断增加
   │
   ▼
机器人追踪指定时间对应的路径点
```

这种方式存在严重问题。

如果机器人因为以下原因没有及时跟上：

```text
速度限制
地面打滑
机器狗步态响应延迟
角速度不足
障碍物避让
网络通信延迟
```

目标点仍然会不断向前移动。

最终会出现：

```text
目标点跑到前方很远的位置
机器人直接朝目标点走直线
机器人切弯
机器人偏离原始路径
机器人撞向障碍物
```

正确方式应该是：

```text
根据机器人当前位置搜索最近路径点，
然后从最近路径点沿路径向前选择目标点。
```

---

## 5. 最近路径点搜索

假设全局路径由一系列离散点组成：

```text
P0, P1, P2, ..., Pn
```

每个路径点包含：

```text
x
y
yaw
```

机器人当前位置为：

```text
robot_x
robot_y
robot_yaw
```

最近点可以通过欧氏距离搜索：

```math
d_i = \sqrt{(x_i-x)^2+(y_i-y)^2}
```

选择距离最小的路径点：

```math
i_{nearest} = \arg\min_i d_i
```

工程上不建议每次搜索整条路径。

可以只从上一次最近点附近向前搜索：

```text
搜索范围：

last_nearest_index - 5
到
last_nearest_index + 50
```

这样可以降低计算量，也可以避免最近点索引突然跳到路径的其他位置。

---

## 6. 前视点选择

找到最近路径点后，不要直接追踪最近点。

应该沿路径向前寻找一个前视点。

前视距离记为：

```math
L_d
```

从最近点开始累计路径长度：

```math
s = 0
```

直到：

```math
s \geq L_d
```

将该路径点作为前视点。

前视距离可以设计为与速度相关：

```math
L_d = L_{min} + k_v |v_x|
```

例如：

```text
Lmin = 0.4 m
kv   = 0.8 s
```

当速度较低时：

```text
Ld 较小
跟踪精度较高
```

当速度较高时：

```text
Ld 较大
轨迹更加平滑
```

推荐初始范围：

```text
低速巡检：0.4 ～ 0.8 m
普通行走：0.6 ～ 1.2 m
高速行走：1.0 ～ 2.0 m
```

---

## 7. 将前视点转换到机器人坐标系

机器人当前位姿：

```text
x
y
yaw
```

前视点：

```text
target_x
target_y
```

世界坐标系误差：

```math
dx = target_x - x
```

```math
dy = target_y - y
```

转换到机器人机体坐标系：

```math
x_b = \cos(\theta)dx + \sin(\theta)dy
```

```math
y_b = -\sin(\theta)dx + \cos(\theta)dy
```

其中：

```text
xb：目标点位于机器人前方的距离
yb：目标点位于机器人左侧或右侧的距离
```

通常约定：

```text
yb > 0：目标在机器人左侧
yb < 0：目标在机器人右侧
```

角速度正负方向必须与机器人底层接口定义保持一致。

在 ROS 标准坐标系中通常为：

```text
angular.z > 0：逆时针旋转，也就是向左转
angular.z < 0：顺时针旋转，也就是向右转
```

---

## 8. Pure Pursuit 曲率计算

Pure Pursuit 根据前视点计算机器人需要行驶的曲率。

曲率公式：

```math
\kappa = \frac{2y_b}{x_b^2+y_b^2}
```

角速度前馈项：

```math
\omega_{pp} = v_x \kappa
```

因此：

```text
目标点在左侧：yb > 0，机器人向左转
目标点在右侧：yb < 0，机器人向右转
目标点在正前方：yb ≈ 0，机器人直行
```

需要防止分母接近零：

```cpp
double distance_square = xb * xb + yb * yb;

if (distance_square < 1e-6) {
    curvature = 0.0;
} else {
    curvature = 2.0 * yb / distance_square;
}
```

---

## 9. 路径航向误差

仅使用 Pure Pursuit，机器人能够跟随路径，但机身方向可能不会严格对齐路径切线。

因此还需要计算航向误差。

最近路径点或前视路径点的方向为：

```text
path_yaw
```

机器人当前方向为：

```text
robot_yaw
```

航向误差：

```math
e_\theta = normalize(path\_yaw - robot\_yaw)
```

角度必须归一化到：

```math
[-\pi,\pi]
```

示例函数：

```cpp
double NormalizeAngle(double angle)
{
    while (angle > M_PI) {
        angle -= 2.0 * M_PI;
    }

    while (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }

    return angle;
}
```

---

## 10. 最终角速度控制

推荐将 Pure Pursuit 前馈项与航向误差反馈项结合：

```math
\omega_{cmd}
=
v_x\kappa
+
k_\theta e_\theta
```

其中：

```text
vx * kappa：根据前方曲率提前转弯
k_theta * e_theta：让机身朝向贴合路径方向
```

可以进一步加入横向误差反馈：

```math
\omega_{cmd}
=
v_x\kappa
+
k_\theta e_\theta
+
k_y \arctan\left(\frac{e_y}{L_d}\right)
```

其中：

```text
ey：机器人到路径的有符号横向误差
```

推荐先从较简单的形式开始：

```math
\omega_{cmd}
=
v_x\kappa
+
k_\theta e_\theta
```

初始参数可以尝试：

```text
k_theta = 0.8 ～ 1.5
```

如果机器人机身方向调整过慢，可以增大 `k_theta`。

如果机器人左右摆动明显，可以减小 `k_theta`。

---

## 11. 航向误差大时先转向

如果机器人当前方向和路径方向差异很大，不应该继续向前冲。

例如：

```text
路径方向向前
机器人机身朝向侧面
```

此时继续发送较大前向速度，机器人容易：

```text
切弯
走斜线
冲出路径
撞向障碍物
```

因此应该根据航向误差降低前向速度。

一种连续控制方式：

```math
v_x =
v_{ref}
\max(0,\cos(e_\theta))
```

对应关系：

```text
航向误差 0°：正常前进
航向误差 30°：轻微减速
航向误差 60°：明显减速
航向误差 90°：停止前进
```

也可以使用分段控制：

```cpp
double abs_heading_error = std::abs(heading_error);

if (abs_heading_error > 1.0) {
    // 大约 57 度
    vx_cmd = 0.0;
} else if (abs_heading_error > 0.6) {
    // 大约 34 度
    vx_cmd *= 0.2;
} else if (abs_heading_error > 0.3) {
    // 大约 17 度
    vx_cmd *= 0.6;
}
```

推荐状态逻辑：

```text
航向误差大于 60°：
原地旋转，vx = 0

航向误差在 30° ～ 60°：
低速前进并转向

航向误差小于 30°：
正常路径跟踪
```

---

## 12. 曲率限速

机器狗在直线和弯道上不应该使用相同速度。

曲率越大，转弯越急，前向速度应该越低。

简单限速公式：

```math
v_{curve}
=
\frac{v_{max}}{1+k_\kappa|\kappa|}
```

最终速度：

```math
v_x
=
\min(v_{ref},v_{curve})
```

也可以根据最大横向加速度限制：

```math
v_{curve}
=
\sqrt{\frac{a_{lat,max}}{|\kappa|+\epsilon}}
```

最终速度：

```math
v_x
=
\min(v_{max},v_{curve})
```

推荐初始速度范围：

```text
直线：0.6 ～ 1.0 m/s
普通弯道：0.3 ～ 0.6 m/s
急弯：0.1 ～ 0.3 m/s
```

对于物业巡检机器狗，建议先保守设置：

```text
最大前向速度：0.6 m/s
普通巡检速度：0.3 ～ 0.5 m/s
急弯速度：0.15 ～ 0.25 m/s
```

---

## 13. 横移速度控制策略

正常路径跟踪时：

```text
vy = 0
```

不建议始终使用：

```math
v_y = -k_y e_y
```

因为这样会导致机器狗：

```text
身体朝向不变
横着贴回路径
运动姿态不自然
朝向与路径方向不一致
避障轨迹不稳定
```

更好的方式是：

```text
优先使用角速度调整机身方向，
再通过前向运动逐渐消除横向误差。
```

---

## 14. 横移恢复模式

横移能力不建议完全删除。

可以保留为异常恢复手段。

推荐设置三种状态：

```text
ALIGN
TRACK
RECOVERY
```

### ALIGN：方向对齐模式

进入条件：

```text
|heading_error| > 60°
```

控制输出：

```text
vx = 0
vy = 0
wz = k_theta * heading_error
```

目标：

```text
先原地旋转，让机身朝向路径方向。
```

---

### TRACK：正常跟踪模式

进入条件：

```text
航向误差较小
横向误差在允许范围内
```

控制输出：

```text
vx > 0
vy = 0
wz = Pure Pursuit + 航向反馈
```

目标：

```text
使用前进和转向沿路径运动。
```

---

### RECOVERY：横移恢复模式

进入条件可以设置为：

```text
横向误差超过 0.5 m
持续一定时间无法靠转向恢复
机器人处于狭窄空间
局部避障明确要求侧移
```

横移速度：

```math
v_y
=
clip(-k_{vy}e_y,-v_{y,max},v_{y,max})
```

推荐限制：

```text
最大横移速度：0.05 ～ 0.10 m/s
```

恢复到路径附近后：

```text
重新切换到 TRACK 模式
vy 恢复为 0
```

---

## 15. 速度平滑

不能直接将当前计算速度发送给机器人。

例如：

```text
上一周期 vx = 0.2 m/s
下一周期 vx = 0.8 m/s
```

这会导致机器狗突然加速。

需要限制线加速度。

线速度限制：

```math
v_{x,k}
=
clip
\left(
v_{x,target},
v_{x,k-1}-a_{dec}\Delta t,
v_{x,k-1}+a_{acc}\Delta t
\right)
```

示例代码：

```cpp
double LimitVelocity(
    double target,
    double previous,
    double acceleration_limit,
    double deceleration_limit,
    double dt)
{
    double delta = target - previous;

    double max_increase = acceleration_limit * dt;
    double max_decrease = deceleration_limit * dt;

    if (delta > max_increase) {
        return previous + max_increase;
    }

    if (delta < -max_decrease) {
        return previous - max_decrease;
    }

    return target;
}
```

推荐初始参数：

```text
最大前向加速度：0.3 ～ 0.5 m/s²
最大前向减速度：0.5 ～ 0.8 m/s²
```

---

## 16. 角速度平滑

角速度也需要限制变化率。

```math
\omega_k
=
clip
\left(
\omega_{target},
\omega_{k-1}-a_\omega\Delta t,
\omega_{k-1}+a_\omega\Delta t
\right)
```

示例代码：

```cpp
double LimitAngularVelocity(
    double target,
    double previous,
    double angular_acceleration_limit,
    double dt)
{
    double max_delta = angular_acceleration_limit * dt;
    double delta = target - previous;

    if (delta > max_delta) {
        return previous + max_delta;
    }

    if (delta < -max_delta) {
        return previous - max_delta;
    }

    return target;
}
```

推荐初始参数：

```text
最大角速度：0.8 ～ 1.2 rad/s
最大角加速度：0.8 ～ 1.5 rad/s²
```

---

## 17. 控制周期

路径跟踪控制器建议运行频率：

```text
20 Hz ～ 50 Hz
```

推荐初始值：

```text
控制频率：30 Hz
dt = 0.033 s
```

频率过低可能导致：

```text
转向滞后
轨迹不平滑
弯道切角
速度指令跳变
```

频率过高但定位更新速度不足，也可能导致：

```text
重复使用旧位姿
控制噪声增加
角速度抖动
```

---

## 18. 参考伪代码

```cpp
void PathTracker::Update()
{
    // 1. 获取当前机器人位姿
    Pose2D robot_pose = GetRobotPose();

    // 2. 搜索最近路径点
    int nearest_index = FindNearestPathPoint(
        robot_pose,
        last_nearest_index);

    last_nearest_index = nearest_index;

    // 3. 找前视点
    double lookahead_distance =
        min_lookahead +
        lookahead_gain * std::abs(last_vx);

    int target_index = FindLookaheadPoint(
        nearest_index,
        lookahead_distance);

    PathPoint target_point = path[target_index];
    PathPoint nearest_point = path[nearest_index];

    // 4. 目标点转换到机器人坐标系
    double dx = target_point.x - robot_pose.x;
    double dy = target_point.y - robot_pose.y;

    double cos_yaw = std::cos(robot_pose.yaw);
    double sin_yaw = std::sin(robot_pose.yaw);

    double xb = cos_yaw * dx + sin_yaw * dy;
    double yb = -sin_yaw * dx + cos_yaw * dy;

    // 5. 计算 Pure Pursuit 曲率
    double distance_square = xb * xb + yb * yb;

    double curvature = 0.0;

    if (distance_square > 1e-6) {
        curvature = 2.0 * yb / distance_square;
    }

    // 6. 航向误差
    double heading_error = NormalizeAngle(
        nearest_point.yaw - robot_pose.yaw);

    // 7. 基础前向速度
    double vx_target = reference_speed;

    // 8. 曲率限速
    double curve_speed =
        max_speed /
        (1.0 + curvature_speed_gain * std::abs(curvature));

    vx_target = std::min(vx_target, curve_speed);

    // 9. 根据航向误差减速
    double abs_heading_error = std::abs(heading_error);

    if (abs_heading_error > 1.0) {
        vx_target = 0.0;
    } else if (abs_heading_error > 0.6) {
        vx_target *= 0.2;
    } else if (abs_heading_error > 0.3) {
        vx_target *= 0.6;
    }

    // 10. 计算角速度
    double wz_target =
        vx_target * curvature +
        heading_gain * heading_error;

    wz_target = std::clamp(
        wz_target,
        -max_angular_velocity,
        max_angular_velocity);

    // 11. 正常情况下不横移
    double vy_target = 0.0;

    // 12. 异常恢复时允许少量横移
    double lateral_error = ComputeLateralError(
        robot_pose,
        nearest_point);

    if (std::abs(lateral_error) > recovery_error_threshold &&
        abs_heading_error < recovery_heading_threshold) {

        vy_target = std::clamp(
            -lateral_recovery_gain * lateral_error,
            -max_lateral_velocity,
            max_lateral_velocity);
    }

    // 13. 加速度限制
    double vx_cmd = LimitVelocity(
        vx_target,
        last_vx,
        max_acceleration,
        max_deceleration,
        control_dt);

    double wz_cmd = LimitAngularVelocity(
        wz_target,
        last_wz,
        max_angular_acceleration,
        control_dt);

    // 14. 发布速度
    geometry_msgs::msg::Twist cmd_vel;

    cmd_vel.linear.x = vx_cmd;
    cmd_vel.linear.y = vy_target;
    cmd_vel.angular.z = wz_cmd;

    cmd_vel_pub_->publish(cmd_vel);

    last_vx = vx_cmd;
    last_wz = wz_cmd;
}
```

---

## 19. 推荐初始参数

```yaml
controller_frequency: 30.0

max_linear_velocity: 0.6
reference_velocity: 0.4
max_lateral_velocity: 0.08
max_angular_velocity: 1.0

max_linear_acceleration: 0.4
max_linear_deceleration: 0.7
max_angular_acceleration: 1.2

min_lookahead_distance: 0.5
lookahead_velocity_gain: 0.8

heading_gain: 1.0
curvature_speed_gain: 1.5

align_heading_threshold: 1.0
slow_heading_threshold: 0.6
medium_heading_threshold: 0.3

recovery_error_threshold: 0.5
recovery_heading_threshold: 0.5
lateral_recovery_gain: 0.2
```

---

## 20. 参数调试方法

### 机器人经常切弯

可以：

```text
减小最大前向速度
增大曲率限速系数
减小前视距离
增大最大角速度
增大 heading_gain
```

---

### 机器人左右摆动

可以：

```text
增大前视距离
减小 heading_gain
降低最大角速度
增加角加速度平滑
对路径 yaw 或曲率进行滤波
```

---

### 机器人转向太慢

可以：

```text
增大 heading_gain
增大最大角速度
增大最大角加速度
降低弯道前向速度
```

---

### 机器人斜着走

检查：

```text
vy 是否始终被设置为 0
机器人底层是否存在自动横移控制
机体坐标系方向是否正确
路径 yaw 是否正确
angular.z 正负号是否正确
```

---

### 机器人落后后突然冲向远处

检查：

```text
是否仍然按照时间推进路径点
是否使用当前位置搜索最近路径点
前视点是否从最近路径点开始计算
最近点索引是否发生异常跳变
```

---

## 21. 路径本身也需要平滑

控制器只能跟踪已有路径。

如果原始路径存在：

```text
尖角
路径点间距不均匀
方向突然变化
重复路径点
路径自交
yaw 跳变
```

即使控制器设计正确，机器人仍然可能运动不平滑。

推荐在跟踪前对路径进行：

```text
路径重采样
B 样条平滑
Bezier 曲线平滑
Savitzky-Golay 平滑
曲率连续优化
```

路径点间距建议：

```text
0.05 ～ 0.15 m
```

对于机器狗低速巡检，可以使用：

```text
0.1 m 左右
```

---

## 22. 最终建议

四足机器狗路径跟踪建议遵循以下原则：

```text
1. 不按照时间推进目标点
2. 根据当前位置搜索最近路径点
3. 使用前视点进行 Pure Pursuit 跟踪
4. 使用路径航向误差修正机身方向
5. 航向误差大时先转向，再前进
6. 根据曲率自动降低弯道速度
7. 正常情况下设置 vy = 0
8. 只有异常恢复时允许少量横移
9. 对线速度和角速度进行变化率限制
10. 对原始路径进行平滑和等距离重采样
```

核心控制思想：

```text
优先通过角速度调整机身朝向，
再通过前向速度消除位置误差，
不要优先依靠横向平移纠正路径。
```

最终正常控制输出：

```text
vx：根据曲率和航向误差动态调整
vy：正常情况下为 0
wz：Pure Pursuit 前馈 + 航向误差反馈
```

这样可以让四足机器狗在保持机身朝向自然的情况下，平滑地沿曲线路径运动。