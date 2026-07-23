# SCAN-Planner 算法与 ROS 2 代码解析

> 整理日期：2026-07-23  
> 对应分支：`ros2-community`  
> 对应代码提交：`948621c`（工作区另有未提交的 `controllers.yaml` 修改，不纳入本文结论）  
> 论文：Han Zheng 等，*SCAN-Planner: Spatial Collision-Aware Local Planning for Route-Guided Long-Range Quadruped Navigation*，arXiv:2606.19555v1

## 1. 一句话理解

SCAN-Planner 是一个面向四足机器人的**在线局部轨迹规划器**。它不负责步态和落足点规划，也不单独完成全局建图或全局路线搜索；它接收点云、里程计和目标/粗略路线，在机器人周围维护一张高分辨率 3D 滑动占据地图，并持续输出可供底层控制器跟踪的三次均匀 B-spline。

它最核心的设计是：

1. 用沿机身前后布置的两个竖直圆柱近似长条形机身；
2. 从轨迹切线推导机身 yaw，不把 yaw 作为额外优化变量；
3. 发现初始轨迹穿过障碍时，用限制在斜平面上的 A* 找一条绕障引导路径；
4. 将 A* 路径转换为“反弹方向”，用 L-BFGS 优化 B-spline；
5. 固定优化过程中的 z 分量，使轨迹主要在水平面内绕障，同时保留上楼梯或斜坡所需的高度趋势；
6. 通过局部滑动地图和周期性重规划支持长距离运行。

因此，它可以概括为：

```text
粗略全局路线
    ↓ 截取局部目标
多项式/B-spline 初值
    ↓ 双圆柱碰撞检测
碰撞段的投影 A* 引导
    ↓ 构造反弹约束
平滑性 + 碰撞 + 动力学代价优化
    ↓ 可行性检查与时间重分配
局部 B-spline
    ↓
机器狗轨迹跟踪/速度控制
```

## 2. 要解决的三个问题

论文认为，四足机器人局部导航与常见二维移动机器人或无人机规划有三个显著区别。

### 2.1 长机身的碰撞范围与 yaw 有关

若只把机器狗当成一个点或圆：

- 按机身宽度膨胀障碍，直行时合理，但转弯时机头或机尾可能扫到障碍；
- 按机身长度膨胀障碍，又会过于保守，许多实际能通过的窄通道会被判死。

若直接把 `(x, y, z, yaw)` 全部纳入搜索和优化，碰撞描述更精确，但计算量明显增加。SCAN-Planner 的折中方案是“双圆柱 + 切线 yaw”：仍然只优化位置轨迹，但用位置轨迹的方向估算机身朝向。

### 2.2 需要 3D 障碍表达，但不能像无人机一样任意上下绕障

二维地图无法区分墙、台阶和低矮可跨越障碍；2.5D 高程图也很难同时表达桌面、桌下空间、货架等悬空结构。

完整 3D 规划虽然能表达这些场景，但如果像无人机规划器一样允许自由改变 z，优化器可能通过“向上飞越障碍”来缩短路径，这对接触地面运动的四足机器人并不可执行。

SCAN-Planner 因此保留 3D 占据地图和 3D 高度曲线，但将绕障搜索限制在由起终点高度插值得到的斜面上，并把优化梯度的 z 分量清零，使绕障主要发生在 xy 平面。

### 2.3 长距离导航不能无限扩大局部地图

高分辨率 3D 栅格的内存开销很大，不适合为数百米路线维护一张同分辨率的全局地图。因此系统只维护机器人附近的固定大小滑动窗口，由低分辨率或外部全局路线提供方向。

局部规划器不断从全局参考轨迹中截取前视目标，随着机器人运动平移地图窗口并重规划。

## 3. 输入、输出和系统边界

### 3.1 算法输入

| 输入 | ROS 2 内部相对话题 | 类型 | 用途 |
|---|---|---|---|
| 机身里程计 | `body_pose` | `nav_msgs/msg/Odometry` | FSM、轨迹起点、控制反馈、滑动地图中心 |
| 传感器位姿 | `sensor_pose` | `nav_msgs/msg/Odometry` | 点云射线原点和坐标变换 |
| 激光点云 | `cloud` | `sensor_msgs/msg/PointCloud2` | 3D 概率占据更新 |
| 深度图 | `depth` | `sensor_msgs/msg/Image` | 可选的深度相机建图输入 |
| 单目标 | `move_base_simple/goal` | `geometry_msgs/msg/PoseStamped` | `navi_mode=1` |
| 预设路点 | `fsm.waypoints` 参数 | xyz 数组 | `navi_mode=2` |
| 外部参考路径 | `initial_path` | `nav_msgs/msg/Path` | `navi_mode=3` |

实际话题由 `src/planner/plan_manage/launch/run.launch.py` 重映射。真机默认值是：

- `/state_estimation`：机身和传感器里程计；
- `/cloud_registered`：已经位于世界坐标系的点云；
- `/scan_planner/cmd_vel`：控制输出；
- `odom`：规划和地图世界坐标系。

这些只是启动文件默认值，并不是算法硬编码接口。移植时应通过 launch 参数或 remap 接到机器狗自己的定位、点云和控制话题。

### 3.2 算法输出

核心规划输出是：

- `planning/bspline`：控制点、节点向量、阶次、轨迹编号和起始时间；
- `planning/data_display`：规划状态和调试数据；
- RViz 可视化话题：初始路径、A* 路径、优化轨迹、占据栅格、膨胀栅格等。

本 ROS 2 分支附带的闭环控制器订阅 `planning/bspline`，输出 `geometry_msgs/msg/Twist`。论文的核心贡献到 B-spline 输出为止；具体 `cmd_vel` 跟踪器和 ZsiBot 桥接属于当前仓库的工程扩展。

## 4. 3D 概率占据地图

### 4.1 基于射线的 log-odds 更新

地图把空间离散成分辨率为 `resolution` 的体素。对每个点云点，从传感器原点向点云终点做 raycast：

- 射线穿过的体素记为 miss，降低占据概率；
- 有效量程内的射线终点记为 hit，提高占据概率；
- 概率被 `p_min` 和 `p_max` 截断；
- 高于 `p_occ` 的体素才被认为是障碍。

代码先将概率转换为 log-odds，避免重复贝叶斯更新时频繁做乘除。对应实现位于：

- 参数及 log-odds 初始化：`plan_env/src/grid_map.cpp:18-122`；
- 射线遍历和命中/空闲缓存：`plan_env/src/grid_map.cpp:542-705`；
- 点云回调及坐标处理：`plan_env/src/grid_map.cpp:780-930`。

这种带上下限的概率更新有两个效果：

- 固定墙体会通过多次 hit 稳定保留；
- 人或临时障碍离开后，后续 miss 能逐步清空旧占据。

它适合缓慢动态环境，但不是带速度估计和未来预测的动态障碍规划器。

### 4.2 障碍膨胀

每当某个体素跨过占据阈值，代码不会重建整张膨胀地图，而是增量更新该体素周围的一组预计算偏移：

- xy 平面按 `double_cylinder_radius` 做圆形膨胀；
- z 方向向上膨胀 `obstacles_inflation_z_up`；
- z 方向向下膨胀 `obstacles_inflation_z_down`。

代码位置：

- 膨胀偏移构造：`grid_map.cpp:208-226`；
- 增量引用计数：`grid_map.cpp:259-299`。

膨胀层使用计数器而不是单个布尔值，因为一个膨胀体素可能同时被多个原始障碍覆盖。只有覆盖计数降到 0，它才会恢复为空闲。

这里的 z 向上下膨胀不是普通对称球形膨胀。论文中的含义是：

- `d_up` 覆盖机身和搭载设备的上方空间，从而正确判断能否钻过桌面或横梁；
- `d_down` 约等于机身中心高度减去可跨台阶高度，使低于可跨越能力的障碍不一定阻塞机身中心轨迹。

当前代码没有单独的 `d_step` 参数，而是直接由使用者把最终期望值折算到 `obstacles_inflation_z_down`。

### 4.3 机器人中心滑动地图

地图尺寸固定为：

```text
sliding_map_size_x × sliding_map_size_y × sliding_map_size_z
```

当地图中心相对机器人偏移超过 `map_sliding_thresh` 时，地图在全局体素索引上平移：

- 新旧窗口重叠区域保留；
- 离开窗口的地址被清理；
- 这些循环缓冲区地址随后代表新进入窗口的体素；
- 若一次位移超过整个地图尺寸，则直接清空整张地图。

核心代码在 `grid_map.cpp:324-421`。`toAddress()` 通过取模把全局体素索引映射到固定大小数组，因而不需要搬移重叠区域的数据。这与论文图 5 描述的 A/B/C 三个区域一致。

`map_min_boundary_` 和 `map_max_boundary_` 是**当前滑动窗口的世界坐标边界**，会随 `map_origin_idx_` 改变，并不是启动后固定的全局地图范围。

### 4.4 未知和越界空间

当前实现中：

- 未观测体素的 log-odds 低于 `p_min`，在普通碰撞查询中不会直接当作障碍；
- 超出滑动地图边界时，`getInflateOccupancyFromBuffer()` 返回 `-1`；
- 调用者通常用 `if (occupancy)` 判断，所以越界会被当作不可通行。

这意味着“地图内部未知”较乐观，“地图外部”较保守。实际部署时必须让地图范围、点云量程和规划前视距离相互匹配。

## 5. 航向相关双圆柱碰撞模型

### 5.1 几何模型

机器人中心为 \(Q_k\)，轨迹局部方向给出的 yaw 为 \(\psi_k\)。前后两个圆柱中心在机体系中是：

\[
s_{b,1}=[d_{off},0,0]^T,\quad
s_{b,2}=[-d_{off},0,0]^T
\]

转换到世界系：

\[
s_{k,j}=R_z(\psi_k)s_{b,j}+Q_k,\quad j\in\{1,2\}
\]

障碍已经按单个圆柱半径和上下高度完成膨胀，因此整机碰撞只需查询两个圆柱中心：

\[
H_k(Q_k,\psi_k)=\max_j \chi(s_{k,j})
\]

两个查询都为空闲，才认为机器人在该位置和朝向下无碰撞。

代码直接对应 `plan_env/include/plan_env/grid_map.h:371-388`：

```cpp
heading = [cos(yaw), sin(yaw), 0]
front = pos + offset * heading
rear  = pos - offset * heading
```

### 5.2 yaw 如何获得

优化变量中没有 yaw 控制点。控制点 \(Q_i\) 的估计航向来自相邻控制点的中心差分：

\[
\psi_i=\operatorname{atan2}
\left((Q_{i+1}-Q_{i-1})_y,(Q_{i+1}-Q_{i-1})_x\right)
\]

不同阶段使用的近似略有区别：

- 控制点检查：前后控制点中心差分；
- 线段密集采样：当前线段方向；
- A* 扩展：当前 `(dx, dy)` 邻接方向；
- 最终轨迹安全检查：相邻时间采样点的切线方向；
- 控制执行：轨迹前视方向或速度方向。

这使“碰撞检测假设的机身朝向”和“控制器希望机器狗跟踪的朝向”大体一致，同时避免把 yaw 加入 L-BFGS 变量。

### 5.3 模型能力与局限

优点：

- 比单圆模型更接近长条机身；
- 每次只需查询两个点，计算开销低；
- 转弯时能够反映机头/机尾扫掠风险；
- 可利用较窄但朝向合适的通道。

局限：

- 只检查离散轨迹点/采样点，不是严格连续碰撞检测；
- 两圆柱是近似包络，无法精确描述腿部摆动、尾部或异形载荷；
- yaw 完全由路径切线决定，不支持“侧身横移通过狭窄区域”这种位置方向与机身方向解耦的动作；
- roll 和 pitch 不进入机身碰撞模型，在大坡度、楼梯边缘和侧倾场景中仍是近似；
- 膨胀参数是静态值，不会根据步态、机身高度或姿态在线变化。

## 6. 从全局路线截取局部目标

SCAN-Planner 不是完整全局规划器。本分支先根据输入目标或路径生成一条无障碍约束的全局多项式参考轨迹，然后每次重规划从中取一个局部目标。

`SCANReplanFSM::getLocalTarget()` 从上一次全局进度开始向前采样，选择距离当前局部起点约 `planning_horizon` 的点：

- 距最终目标仍远时，局部目标速度沿用全局参考速度；
- 接近最终目标、剩余距离小于制动距离时，局部目标速度设为 0；
- 如果局部目标已占据，会沿全局轨迹前后搜索附近的空闲点。

对应代码为 `scan_replan_fsm.cpp:1012-1113`。

需要注意，“全局轨迹”在不同导航模式中的含义不同：

- 模式 1：从当前位置到 RViz 单目标的平滑参考线；
- 模式 2：按预设 waypoint 逐段导航；
- 模式 3：接收 `initial_path`，把外部路线作为引导；
- 它本身不保证避障，真正绕障由局部规划完成。

## 7. 局部轨迹生成

### 7.1 三次均匀 B-spline

位置轨迹 \(p(t)\in \mathbb{R}^3\) 使用三次均匀 B-spline 表示。若控制点为 \(Q_i\)，节点间隔为 \(\Delta t\)，则速度、加速度、jerk 控制点可由相邻差分得到：

\[
V_i=\frac{Q_{i+1}-Q_i}{\Delta t}
\]

\[
A_i=\frac{V_{i+1}-V_i}{\Delta t}
\]

\[
J_i=\frac{A_{i+1}-A_i}{\Delta t}
\]

三次 B-spline 的首尾各 3 个控制点用于确定边界位置、速度和加速度；优化器只改变中间控制点。

### 7.2 初始轨迹

每轮局部规划首先构造一条连接当前状态和局部目标状态的多项式：

- 正常情况使用单段多项式；
- 连续规划失败时，可以插入随机中间点生成另一条 min-snap 初值；
- 重规划时也可从上一条未执行完的 B-spline 取样，再拼接到新局部目标。

采样间隔根据：

- `manager.control_points_distance`；
- `manager.max_vel`；
- 起终点距离和初始轨迹采样间距

自适应调整，且至少保证 7 个样本点。

### 7.3 高度正则化

多项式初值可能出现不必要的 z 起伏。代码在 B-spline 参数化前，保留每个样本的 xy，并按累计水平弧长在线性高度曲线上重新赋值：

\[
\ell_i=\sum_{m=1}^{i}\|r_{m,xy}-r_{m-1,xy}\|,\quad L=\ell_N
\]

\[
z_i=z_s+\frac{\ell_i}{L}(z_g-z_s)
\]

若水平总长度接近 0，则改用样本序号比例。

对应 `planner_manager.cpp:9-38` 和 `planner_manager.cpp:251`。这一步只提供名义高度趋势，例如从一层 waypoint 到楼梯上方 waypoint 时形成平滑上升趋势；它本身并不从地形估计可行地面高度。

## 8. 投影 A* 和反弹约束

### 8.1 识别碰撞段

`BsplineOptimizer::initControlPoints()` 沿初始控制点线段按不大于约半个体素的间距采样，用双圆柱模型检查占据状态，并把连续碰撞区间分割为若干 `[in_id, out_id]`。

只有检测到碰撞的区间才调用 A*。这继承了 EGO-Planner 的“lazy rebound”思想：不维护完整 ESDF，也不对所有控制点计算障碍距离。

### 8.2 投影到斜面的 A*

普通 3D A* 会扩展 26 邻域并允许从障碍上方绕过。SCAN-Planner 只在 xy 上扩展 8 邻域，每个候选节点的 z 由碰撞段入口和出口线性插值：

\[
\beta(x_{xy})=
\Pi_{[0,1]}
\left(
\frac{(x_{xy}-x^{in}_{xy})^T(x^{out}_{xy}-x^{in}_{xy})}
{\|x^{out}_{xy}-x^{in}_{xy}\|^2}
\right)
\]

\[
z(x_{xy})=z^{in}+\beta(x_{xy})(z^{out}-z^{in})
\]

候选边的 yaw 使用：

\[
\psi_\Delta=\operatorname{atan2}(\Delta y,\Delta x)
\]

只有双圆柱查询为空闲时，邻居才能进入 open set。

对应实现是 `path_searching/src/dyn_a_star.cpp:146-288`。因此该 A* 的作用不是输出最终可执行轨迹，而是给优化器提供碰撞区间的局部绕障拓扑。

### 8.3 从 A* 路径生成反弹方向

对碰撞段内的控制点，优化器寻找控制点局部法平面与 A* 路径的交点，再从控制点向引导路径构造：

- anchor/base point \(a_{ij}\)：靠近障碍边界的基点；
- direction \(v_{ij}\)：把控制点推向 A* 自由路径一侧的单位方向。

控制点到障碍的有符号近似距离为：

\[
d_{ij}=(Q_i-a_{ij})^Tv_{ij}
\]

安全余量误差：

\[
c_{ij}=s_f-d_{ij}
\]

只在安全距离不足时加入单边惩罚。当前代码使用分段三次/二次形式，使代价和梯度连续：

- 误差较小时为三次惩罚；
- 误差较大时切换到匹配的一段二次函数。

对应 `bspline_optimizer.cpp:402-440`。

二值占据查询本身不可导，它只负责决定“哪里碰撞、向哪一侧反弹”；真正交给 L-BFGS 的是上述连续距离近似。

## 9. B-spline 优化目标

第一阶段 rebound 优化的总代价为：

\[
J=\lambda_sJ_s+\lambda_cJ_c+\lambda_fJ_f
\]

### 9.1 平滑代价

\[
J_s=\sum_k\|J_k\|^2
\]

最小化 jerk 控制点平方和，使路径曲率和加速度变化更平滑。

### 9.2 碰撞代价

\[
J_c=\sum_{i,j}\rho(s_f-d_{ij})
\]

使碰撞段控制点沿 A* 生成的反弹方向离开障碍，并保持 `optimization.dist0` 安全距离。

### 9.3 动力学可行性代价

对超过最大速度和最大加速度的分量施加单边平方惩罚：

\[
J_f=\sum_kF(\|V_k\|,v_m)+\sum_kF(\|A_k\|,a_m)
\]

当前实现实际逐轴检查 `vx/vy/vz` 和 `ax/ay/az`，随后还会对完整向量范数做一次采样检查。

### 9.4 z 梯度抑制

代码计算完三类 3D 梯度后执行：

```cpp
grad_3D.row(2).setZero();
```

rebound 和 refine 两阶段都如此，见 `bspline_optimizer.cpp:1167-1170`、`1195-1198`。

这意味着：

- 优化变量数组里仍包含 z；
- 但 L-BFGS 不会更新内部控制点的 z；
- 轨迹 z 主要由高度正则化初值和边界状态决定；
- 绕障只能靠改变 x/y。

这正是论文所谓的 z-gradient suppression。

## 10. 可行性细化与安全检查

完成 rebound 优化后，系统构造最终三次 B-spline 并检查速度、加速度限制：

1. 若超限，按估算比例拉长轨迹时间；
2. 重新参数化控制点；
3. 以时间拉长前的轨迹为参考，进行 smoothness + fitness + feasibility 的 refine 优化；
4. 再次采样检查速度、加速度范数；
5. refine 过程中也会重新检查轨迹前段是否碰撞。

成功后才发布 `planning/bspline`。主要流程位于 `planner_manager.cpp:255-304`。

## 11. 在线重规划状态机

FSM 以 100 Hz 运行，安全检查以 20 Hz 运行。主要状态为：

```text
INIT
  ↓ 已有 odom 和触发目标
WAIT_TARGET
  ↓ 已有目标
GEN_NEW_TRAJ
  ↓ 成功
EXEC_TRAJ
  ├─ 走过 thresh_replan → REPLAN_TRAJ
  ├─ 新障碍且来得及 → REPLAN_TRAJ
  ├─ 新障碍且无法及时重规划 → EMERGENCY_STOP
  └─ 到达终点 → WAIT_TARGET
```

关键行为：

- 执行轨迹离起点超过 `fsm.thresh_replan` 后主动刷新；
- 距最终目标小于 `fsm.thresh_no_replan` 时减少末端反复重规划；
- 安全线程以 0.01 s 间隔扫描当前轨迹有效前段；
- 发现碰撞先立即尝试从当前状态重规划；
- 若碰撞时间小于 `fsm.emergency_time` 且重规划失败，则发布定点 B-spline 急停；
- 多次失败可切换随机多项式初值，尝试逃离局部最小值。

对应 `scan_replan_fsm.cpp:582-750` 和 `877-930`。

## 12. ROS 2 分支的执行和控制扩展

### 12.1 闭环控制器

当前默认闭环控制器提供两种轨迹进度方式：

- `path_follow`：在采样后的 B-spline 上找离当前机器人最近的点，再按空间弧长前视；
- `time`：按轨迹发布时间和系统时间直接取期望状态。

真机默认 `path_follow`，因为机器狗一旦跟踪滞后，纯时间模式会继续追远处的轨迹点并可能切弯。

速度转换也有两种：

- `pure_pursuit`：只输出 `linear.x` 和 `angular.z`；
- `omni`：输出机体系 `linear.x`、`linear.y` 和 `angular.z`。

默认选择 `pure_pursuit`，更适合横移能力弱或 SDK 横移死区明显的机器狗。

当航向误差超过 `heading_error_threshold` 时，控制器冻结轨迹时间推进并先原地转向。控制器通过 `planning/go2_execution_frozen` 告知 FSM 暂停本地轨迹计时，避免“机身还在转，规划轨迹已经走远”。

### 12.2 真机坐标转换

若 LIO 输出的是雷达中心位姿，而规划和控制需要机身中心位姿，可以启用 `lidar_to_body_odom`：

\[
T_{WB}=T_{WS}T_{BS}^{-1}
\]

其中 \(T_{WS}\) 是世界到传感器，\(T_{BS}\) 是机身到传感器外参。对应 launch 参数为 `body_to_sensor_x/y/z/roll/pitch/yaw`。

### 12.3 ZsiBot 执行桥接

仓库提供两种 ZsiBot 接法：

- Orin 直接运行 SDK bridge；
- Orin 发布 UDP，RK3588 本机 proxy 调用 SDK。

它们只负责把 `cmd_vel` 转成厂商高级运动接口，不参与 SCAN-Planner 的地图、碰撞或轨迹优化。

## 13. 论文与当前代码的对应程度

| 论文机制 | 当前代码状态 | 主要证据 |
|---|---|---|
| 三次均匀 B-spline | 已实现 | `uniform_bspline.cpp`、`planner_manager.cpp` |
| 切线诱导 yaw | 已实现 | `estimateControlPointYaw()`、各阶段线段 yaw |
| 双圆柱全身碰撞 | 已实现 | `grid_map.h:371-388` |
| 非对称上下膨胀 | 已实现 | `rebuildInflationOffsets()` |
| 概率占据与 log-odds | 已实现 | `raycastProcess()` |
| 循环缓冲滑动地图 | 已实现 | `updateSlidingMap()`、`toAddress()` |
| 膨胀层增量更新 | 已实现 | `updateInflationLayer()` |
| 高度正则化初值 | 已实现 | `applyLinearZReference()` |
| 投影斜面 A* | 已实现 | `interpolateZIndexOnSearchPlane()` |
| A* 扩展使用方向 yaw | 已实现 | `neighbor_yaw = atan2(dy, dx)` |
| rebound 碰撞约束 | 已实现 | `initControlPoints()`、`calcDistanceCostRebound()` |
| z 梯度抑制 | 已实现 | 两处 `grad_3D.row(2).setZero()` |
| 速度/加速度约束和时间重分配 | 已实现 | `checkFeasibility()`、`refineTrajAlgo()` |
| 轨迹在线安全检查和急停 | 已实现 | `checkCollisionCallback()` |
| 论文式虚拟自由层边界 fallback | **当前代码未发现** | A* 越界直接 `continue`，返回值只有 SUCCESS/INIT_ERR/SEARCH_ERR |
| 地形可通行性/落足点规划 | 不属于本算法，当前核心规划器未实现 | 依赖外部 locomotion 能力和高度引导 |
| 动态障碍速度预测 | 未实现 | 地图只靠概率更新适应变化 |

### 13.1 关于 dead-end recovery 的重要说明

论文第 V-C 节描述：

1. 在滑动地图外临时增加一圈虚拟自由体素；
2. 允许假设路径进入虚拟层；
3. 取路径离开真实地图的位置作为 fallback voxel；
4. 把局部目标替换成边界 fallback；
5. 再在真实地图内生成一段有限恢复运动。

但当前 ROS 2 分支中：

- `dyn_a_star.cpp` 对搜索池边缘节点直接跳过；
- 地图外查询返回 `-1`，被视为不可通过；
- `ASTAR_RET` 没有 fallback 类型，也没有返回边界 voxel 的接口；
- FSM 虽然会把“被占据的局部目标”沿全局参考线前后调整到空闲点，但这不等同于论文的虚拟层边界回退。

因此不能仅根据论文假设当前代码已经具备同等 dead-end recovery 能力。若要复现实验中的长距离死胡同恢复，应补齐这部分并单独测试。

## 14. 参数之间的关键约束

### 14.1 地图尺寸、更新范围与规划前视

当前默认：

```text
planning_horizon = 7.5 m
sliding_map_size_x/y = 10 m
local_update_range_x/y = 5 m
```

若机器人位于地图中心，10 m 地图沿单一方向只有约 5 m 可用，而前视目标可在 7.5 m 外。因此局部目标或初始轨迹可能越过当前地图边界。

虽然碰撞通常发生在轨迹近端、规划器也只重点检查前 2/3，但真机配置仍建议满足：

```text
地图前向有效半径 >= 实际规划前视距离 + 双圆柱偏移 + 安全余量
```

若不想增加 3D 地图内存，应降低 `planning_horizon` 和 `manager.planning_horizon`，并同步考虑雷达可靠量程和 `max_ray_length`。

### 14.2 双圆柱尺寸

近似包络总长度约为：

\[
L_{model}=2(d_{off}+r)
\]

总宽度约为：

\[
W_{model}=2r
\]

参数不能只照抄机身外形：

- `r` 应包含机身半宽、定位误差、点云噪声、控制跟踪误差；
- `d_off` 应让前后圆共同覆盖机身长度；
- 若搭载支架或雷达突出机身，必须纳入包络；
- 控制器前视造成的切弯误差也应由几何安全余量覆盖。

### 14.3 规划速度与执行速度

至少存在三层限速：

1. `manager.max_vel/max_acc`：B-spline 生成和可行性；
2. `closed_loop_controller.max_vx/max_vy/max_vyaw`：ROS 控制器输出；
3. ZsiBot bridge/SDK：厂商接口最终限幅。

若规划器按 0.75 m/s 生成轨迹，而机器狗被限制为 0.2 m/s，`path_follow` 尚能按空间位置追踪，但频繁重规划、制动距离、局部目标速度和动态可行性语义会不一致。正式调试应尽量让规划上限接近真机可稳定达到的速度。

## 15. 移植到另一台机器狗时，算法层面最需要确认的内容

这不是完整部署步骤，而是从算法假设出发的适配清单。

### 15.1 坐标与状态

- 规划世界坐标系、odom `header.frame_id`、点云坐标系和目标坐标系必须一致或存在可靠 TF；
- 明确 LIO 输出的是机身中心还是雷达中心；
- 明确点云是否已在世界系：已注册点云设 `cloud_is_world=true`；
- 若点云在传感器系，必须提供同步传感器位姿和正确外参；
- 四元数、线速度的表达坐标系必须核实，不能只看 topic 名称。

### 15.2 机身几何与能力

- 测量机身和全部载荷的长、宽、高；
- 标定 `double_cylinder_radius` 和 `double_cylinder_offset`；
- 根据机器狗可跨高度设置向下膨胀；
- 根据机身、云台和传感器最高点设置向上膨胀；
- 若机器人不能原地转向或不能横移，使用 pure-pursuit，并增加转角/走廊安全裕量。

### 15.3 感知

- 点云频率和里程计时间戳应稳定；
- 雷达盲区不能覆盖机身即将进入的空间；
- 地面点过滤不应把低矮但不可跨越的障碍全部删掉；
- 稀疏点云下适当降低 `p_occ` 或提高 `p_hit`，但要防止噪声形成幽灵障碍；
- `max_ray_length` 不应超过可靠点云距离，也不能明显小于规划前视。

### 15.4 控制

- 验证 `cmd_vel` 是机体系还是世界系，当前控制器输出机体系速度；
- 验证正方向、角速度单位和 SDK 超时行为；
- 逐层设置规划器、控制器、桥接的速度上限；
- 测量跟踪误差，并把误差加入双圆柱半径安全余量；
- 验证急停时持续发布零速度是否满足底盘接口要求。

### 15.5 地形假设

SCAN-Planner 不判断足端落点是否可行，也不会从原始点云自动生成严格贴地的高度轨迹。楼梯能力依赖：

- 全局路线/waypoint 提供合理的 z 趋势；
- 底层 locomotion controller 自己能够处理台阶；
- 障碍膨胀允许可跨低障碍；
- 路径的坡度和高度变化没有超过机器狗能力。

如果目标机器狗没有成熟的感知运动控制器，只移植 SCAN-Planner 不能自动获得论文中的楼梯通过能力。

## 16. 推荐的阅读顺序

第一次继续开发时，建议按以下顺序看代码：

1. `src/planner/plan_manage/launch/run.launch.py`  
   理解真机/仿真话题、坐标系和节点组成。

2. `src/planner/plan_manage/src/scan_replan_fsm.cpp`  
   理解目标处理、局部目标、重规划、安全检查和轨迹发布。

3. `src/planner/plan_manage/src/planner_manager.cpp`  
   理解初值、高度正则化、B-spline 参数化、优化和时间重分配。

4. `src/planner/plan_env/src/grid_map.cpp` 与 `include/plan_env/grid_map.h`  
   理解概率更新、滑动窗口、膨胀和双圆柱查询。

5. `src/planner/path_searching/src/dyn_a_star.cpp`  
   理解投影 A* 的搜索平面和方向相关碰撞。

6. `src/planner/bspline_opt/src/bspline_optimizer.cpp`  
   理解碰撞分段、反弹方向、代价函数和 z 梯度抑制。

7. `src/planner/plan_manage/src/closed_loop_controller.cpp`  
   理解本 ROS 2 分支如何把论文轨迹变成真机 `cmd_vel`。

## 17. 总结

SCAN-Planner 的关键价值并不是提出一种全新的全局搜索，而是把几项适合四足机器人的局部规划约束组合成一条实时链路：

- 用双圆柱在几乎不增加优化维度的前提下近似全身碰撞；
- 用完整 3D 占据表达悬空和多层结构；
- 用投影 A* 与固定 z 梯度避免无人机式的垂直绕障；
- 用 lazy rebound 在没有 ESDF 的情况下获得可导碰撞代价；
- 用循环缓冲滑动地图把内存限制在固定范围；
- 用高频 FSM、安全扫描和 B-spline 重规划处理在线变化。

对移植而言，算法本体通常不需要首先重写。最影响成败的是四类接口是否满足其假设：

1. 点云、机身位姿和目标是否严格处于一致坐标系；
2. 双圆柱和上下膨胀是否真实覆盖目标机器狗；
3. 规划前视、地图范围和传感器可靠量程是否匹配；
4. B-spline 跟踪器和底盘接口是否能让实际机身朝向跟随轨迹切线。

此外，当前代码未发现论文所述的虚拟层边界 fallback。若目标场景包含长走廊尽头、U 形障碍或局部地图边界死胡同，应把这项缺口列为后续实现和验证重点。

