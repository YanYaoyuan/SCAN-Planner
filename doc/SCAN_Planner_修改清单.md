# SLAM → SCAN-Planner 全局坐标导航链路

## 1. 最终坐标设计

当前重定位只在启动时执行一次 ICP，并锁定静态变换
`lio_map -> lio_odom`。因此真机的建图结果、实时重定位输出、全局参考路径、
局部占据地图、B-spline 和闭环控制统一使用 `lio_map`，可以避免在规划链中
反复转换路径和障碍物坐标。

```text
建图：
FAST-LIO
  ├─ /state_estimation_global    lio_map -> livox_frame
  └─ /cloud_registered_global    PointCloud2(frame=lio_map)

重定位：
FAST-LIO                    ICP/TF
  │ lio_odom -> livox_frame   │ lio_map -> lio_odom
  ├──────────────┬────────────┘
  │              ├─ /state_estimation_global
  │              │     lio_map -> livox_frame
  │              └─ /cloud_registered_global
  │                    PointCloud2(frame=lio_map)
  └─ /state_estimation 与 /cloud_registered 保留为内部/诊断局部输出

规划：
/state_estimation_global
  └─ lidar_to_body_odom
       └─ /body_state_estimation_global
              lio_map -> scan_base_link

/move_base_simple/goal (任意可转换 frame)
  └─ global_path_publisher
       └─ /planning/global_path (lio_map)
              └─ SCAN-Planner + /cloud_registered_global
                    ├─ /planning/bspline (lio_map)
                    └─ closed_loop_controller
                         ├─ /planning/local_path (lio_map)
                         └─ /scan_planner/cmd_vel
```

禁止只修改 PointCloud2 的 `header.frame_id`。全局点云节点会使用点云原始时间戳
查询 `T_lio_map_lio_odom`，并实际变换每个点的 x/y/z。

## 2. 已实现内容

### omni_slam

- 重定位持续发布 `/cloud_registered_global`，同时保留本地
  `/cloud_registered`。
- `/state_estimation_global` 和全局点云都使用同一条锁定的
  `lio_map -> lio_odom` TF。
- 建图启动将 FAST-LIO 的世界 frame 显式设为 `lio_map`，并直接发布
  `/state_estimation_global`、`/cloud_registered_global`。
- 重定位启动显式保持 FAST-LIO 内部 frame 为 `lio_odom`，防止建图和定位模式
  因共用 YAML 而串帧。

### SCAN-Planner

- 新增 `global_path_publisher`：
  - 当前实现从实时全局机体位姿到 RViz 目标生成等间距参考路径；
  - 目标若不是 `lio_map`，先用 TF 转换；
  - 支持参数化的三维 waypoint 序列；
  - 输出 `/planning/global_path`，由 `navi_mode=3` 接收。
- 真机脚本默认切换到：
  - `/state_estimation_global`
  - `/body_state_estimation_global`
  - `/cloud_registered_global`
  - `grid_map.frame_id=lio_map`
- GridMap 使用 ApproximateTime 配对点云与传感器位姿，并检查：
  - 两条消息的时间差不超过 `0.05s`；
  - 点云 frame、里程计 parent frame、child frame 均符合配置。
- `lidar_to_body_odom` 完整处理：
  - 雷达到机体中心的位姿；
  - 带安装杆臂项的线速度；
  - 角速度；
  - pose/twist covariance；
  - 输入 parent/child frame 错误时拒绝发布。
- FSM 和控制器增加 `0.30s` 里程计超时急停、有限值/四元数/frame 校验。
- B-spline 与机体里程计不是同一 frame 时，控制器拒绝执行，不再只报警后继续算。
- 新增 `/planning/local_path`，它是控制器实际跟踪 B-spline 的采样结果。
- 规划前视从 `7.5m` 调整为 `4.0m`，与当前 `10m × 10m` 滑动局部地图匹配。
- 参考路径已表示机体中心高度，取消旧代码重复叠加 `body_height`。
- UDP 与直接 SDK 两个真机脚本使用同一组已标定外参：
  `[0.13011, -0.02329, 0.17598]`。

## 3. 启动方式

先启动 SLAM 重定位，并确认首次 ICP 成功、`lio_map -> lio_odom` 已发布，再启动
SCAN-Planner。

直接 SDK：

```bash
cd /Users/yan/WorkSpace/Omni/SCAN-Planner
./tools/orin_runtime/run_real_planner.sh
```

UDP：

```bash
cd /Users/yan/WorkSpace/Omni/SCAN-Planner
./tools/orin_runtime/run_real_planner_udp.sh
```

然后在 RViz 发布 2D Goal。`global_path_publisher` 会使用当前
`/body_state_estimation_global` 作为起点，发布 `/planning/global_path`。

## 4. 真机运行前检查

所有结果都应满足：

```text
/state_estimation_global.header.frame_id       = lio_map
/state_estimation_global.child_frame_id        = livox_frame
/body_state_estimation_global.header.frame_id  = lio_map
/body_state_estimation_global.child_frame_id   = scan_base_link
/cloud_registered_global.header.frame_id       = lio_map
/planning/global_path.header.frame_id           = lio_map
/planning/bspline.header.frame_id               = lio_map
/planning/local_path.header.frame_id             = lio_map
```

建议低速架空或牵引测试：

1. 原地横移、前进、旋转，检查全局位姿和点云没有相对错位；
2. 发送近距离目标，检查全局路径和局部路径都在 `lio_map`；
3. 运行中停止 SLAM，确认 `0.30s` 内 `/scan_planner/cmd_vel` 变为全零；
4. 人为输入错误 frame 的路径，确认控制器拒绝并保持零速度；
5. ICP 失败或全局 TF 未建立时，规划器不得产生运动命令。

## 5. 当前能力边界

`global_path_publisher` 是全局坐标下的参考路线发布器，不是基于静态 PCD 的
A*/Hybrid-A* 全局避障器。当前静态与临时障碍的绕行仍由 SCAN-Planner 的局部
GridMap 和 B-spline 完成。

如果以后允许运行中重新 ICP 并更新 `lio_map -> lio_odom`，不能直接沿用当前
锁定模型。全局变换发生跳变前必须先急停、清空 GridMap、全局路径和 B-spline，
再从新位姿重新规划。
