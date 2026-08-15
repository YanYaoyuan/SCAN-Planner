<div align="center">
  <h1>SCAN-Planner ROS 2</h1>
  <h2>面向路线引导四足长程导航的空间碰撞感知局部规划器</h2>
  <a href="https://arxiv.org/abs/2606.19555"><img alt="论文" src="https://img.shields.io/badge/论文-arXiv-b31b1b?logo=arxiv&logoColor=white"/></a>
  <a href="https://www.bilibili.com/video/BV15a7P6UEXb/"><img alt="视频" src="https://img.shields.io/badge/视频-Bilibili-FB7299?logo=bilibili&logoColor=white"/></a>
  <a href="https://wuyi2121.github.io/SCAN-Planner/"><img alt="项目主页" src="https://img.shields.io/badge/项目主页-Website-4A90E2?logo=googlechrome&logoColor=white"/></a>
</div>

<p align="center">
  <img src="assets/images/abstract_real.jpg" width="100%"/>
</p>

SCAN-Planner 是一款面向四足机器人导航的空间碰撞感知局部规划器。本分支是原生 ROS 2 自移植版本，适配 Ubuntu 22.04、ROS 2 Humble、C++17 和 `colcon` 构建系统。

本仓库是 [wuyi2121/SCAN-Planner](https://github.com/wuyi2121/SCAN-Planner) 的衍生 ROS 2 移植版。核心算法、项目设计与原始研究工作归功于 Han Zheng、Zhe Chen、Yiwen Fu、Ming Yang 和 Tong Qin；ROS 2 适配由本仓库维护者完成，不代表原作者的官方发布或认可。

## 构建

安装 ROS 2 Humble 及包依赖后，在工作空间根目录下执行构建：

```bash
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -r -y
sudo apt install libarmadillo-dev libglew-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev

colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

默认构建 CPU 端本地感知后端，如需构建 OpenGL 后端可执行：

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DUSE_GPU=ON
```
仓库不再链接自带的 x86_64 架构 GLFW 动态库，GPU 构建依赖系统安装的 GLFW、GLEW 和 OpenGL 相关包。

## 快速启动

启动自定义确定性仿真器与规划器：

```bash
source install/setup.bash
ros2 launch scan_planner run.launch.py \
  is_real_world:=false navi_mode:=1 sensor_type:=lidar \
  controller_mode:=closed_loop use_gpu:=false
```


```bash
source install/setup.bash
ros2 launch scan_planner run.launch.py \
  is_real_world:=false navi_mode:=1 sensor_type:=lidar \
  controller_mode:=closed_loop use_gpu:=false \
  use_pcd_map:=true pcd_map_file:=/home/user/robot/SCAN-Planner/scans.pcd
```



在另一个终端启动 RViz2：

```bash
source install/setup.bash
ros2 launch scan_planner rviz.launch.py
```

## dog3 模式3：全局路径输入、局部路径输出

融合后的 dog3 入口保留 Omni 新版 `SCANReplanFSM`、空间进度、安全心跳和
点云/位姿时间同步，只启动规划节点，不启动 controller、仿真器或
`robot_state_publisher`，因此不会新增 `cmd_vel` 发布端。

dog3 ARM64 板端构建：

```bash
./2_build_dog3.sh
```

板端构建需要已有 Docker、`/app/opt/ros/humble` 和 `/app/idl_msgs`；运行时
还需要 `/app_param`，厂家 `/app/script/env.sh` 存在时会自动加载其 Zenoh 配置。
构建脚本会拒绝非 ARM64 主机，避免产出架构与镜像名不一致的二进制。

构建产物单独放在 `install-dog3/`，不会污染普通 `install/`。检查并启动：

```bash
./1_run_scanplanner.sh --check
./1_run_scanplanner.sh
```

固定接口为：

- 里程计：`/relocalizing/map_frame/odometry`
- 点云：`/lidar_points`
- 全局路径：`/map_frame/global_path`
- 局部路径：`/map_frame/local_path`（`nav_msgs/msg/Path`）
- 新版调试轨迹：`/planning/bspline_v2`

dog3 差异全部位于
`src/planner/plan_manage/config/dog3.yaml`。启动文件会先加载通用
`planner.yaml`，再应用 dog3 覆盖并完成 canonical topic remap。PC 录包回放
也可使用：

```bash
source install/setup.bash
ros2 launch scan_planner dog3_mode3.launch.py \
  use_rviz:=true use_sim_time:=true
```

PC 开启 RViz 时可视化副本会把 `vita_lidar` 标记为与其物理重合的
`lidar_frame`；算法输入仍保持原消息，并继续执行严格 frame 与时间戳检查。
真 dog3 环境强制禁用 RViz。

规划输入继续使用 Omni 的严格同步检查。当前 Dog3 契约要求 odom 的
`header.frame_id=map_frame`、`child_frame_id=lidar_frame`，点云
`header.frame_id=vita_lidar`，两者时间差不超过 0.05 秒；启动日志会打印这组
期望值，并在不匹配时持续报出实际值。首次上板应先用真实话题/rosbag 核对，
如硬件接口不同则修改 `dog3.yaml` 中的显式 frame 或同步容差。
只有确认不同 frame 名物理等价、或确认时间偏差可接受时，才临时关闭
`grid_map.strict_frame_check` / `grid_map.strict_sync_check`；兼容模式仍会持续告警。
占据地图超过 2.0 秒未收到有效同步更新时，规划器会立即清空局部轨迹；新鲜
点云恢复后，会重新验收最后一条已接受的全局路径。freshness 同时检查消息源
时间戳和本机处理时间，executor 阻塞期间积压的旧点云不会因延迟处理而冒充
新地图；首次上板仍需实测正常规划 P99，并让该阈值留有明确余量。
dog3 的 odom watchdog 同样使用 2.0 秒预算并同时检查 source/processing age；
底盘消费端自己的超时仍应更短，不能依赖规划器线程代替最终停车保护。

已核到 Desktop 相邻的 `3_record_publish_path/publish_path.py` 当前把
`/map_frame/global_path` 的消息 `frame_id` 写成 `lio_map`。本融合不会把两个
frame 静默视为相同；上板前必须将发布端改为 `map_frame`，或提供可靠的
`lio_map -> map_frame` TF，否则该全局路径会被拒绝。

Desktop 版的 `scan_planner_msgs/msg/Bspline` 没有 `Header`，与 Omni 新消息
不兼容。因此 dog3 必须全新构建并停止旧 planner endpoint；稳定的模块边界
应使用 `/map_frame/local_path`，新版 B-spline 则隔离在
`/planning/bspline_v2`。

> **下游停车契约（上板硬条件）**：已核到 Desktop/0725 的旧
> `7_moveforward/dog_move_forward.py` 会忽略空 `Path`，并永久复用上一条路径
> 继续发布 `cmd_vel`；它不能直接与本入口搭配。dog3 的实际路径消费端必须在
> 收到空 `/map_frame/local_path` 时立即清缓存并发布零速，还必须使用
> steady-clock 路径接收超时（推荐再加 planner heartbeat 门控）。否则 planner
> 崩溃、odom 超时或点云断流时都无法保证停车。

RViz2 配置已适配 ROS 2 Humble：Go2 的 RobotModel 使用现有的 `meshes/base.dae`，Sliding Map Bounds 订阅 `/grid_map/sliding_map_bbox`，Goal 订阅 `/goal_point`。



导航模式说明：
- `navi_mode:=1`：使用 RViz2 的 2D 目标点工具选择导航目标
- `navi_mode:=2`：按照 ROS 2 参数文件中预设的 `fsm.waypoints` 路径点序列导航
- `navi_mode:=3`：订阅 `initial_path` 话题获取全局路径，并在局部范围内进行避障

控制器模式分为 `open_loop`（开环）和 `closed_loop`（闭环）两种。本次移植保留的核心启动参数包括：`is_real_world`、`navi_mode`、`sensor_type`、`controller_mode`、`use_gpu`、`use_pcd_map` 和 `pcd_map_file`。

当 `use_pcd_map:=true` 时，必须提供已有的 PCD 点云地图文件：

```bash
ros2 launch scan_planner run.launch.py \
  use_pcd_map:=true pcd_map_file:=/absolute/path/to/map.pcd
```

实际硬件部署时，激光惯导里程计（LIO）、相机和宇树（Unitree）驱动均为外部依赖，默认启动会将规划器输入映射到 `/LIO/odom_vehicle`、`/LIO/odom_imu`、`/LIO/clouds_lidar` 话题以及 RealSense 对齐深度图话题，可根据实际安装的驱动栈修改话题重映射配置。

### 旧 ZsiBot 机器狗桥接（仅迁移兼容）

`zsibot_cmd_bridge` 已进入下线阶段。SCAN-Planner 默认只向隔离话题
`/scan_planner/cmd_vel` 发布速度，不启动旧桥、不持有厂商 SDK，也不再声明旧桥为
运行依赖。产品部署应由统一 robot bridge 独占厂商 SDK。

仅在迁移或隔离台架测试时显式编译旧包。Orin 侧推荐的 UDP client
不持有厂商 SDK，因此保持 SDK targets 关闭：

```bash
colcon build --symlink-install --packages-select zsibot_cmd_bridge \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DZSIBOT_ENABLE_DEPRECATED_SDK_TARGETS=OFF
```

只有 RK proxy 或 direct bridge 的隔离构建才使用
`-DZSIBOT_ENABLE_DEPRECATED_SDK_TARGETS=ON`；这些产物不得进入默认产品包。
同机 SDK owner 会竞争 `/run/lock/omni/zsibot_sdk_owner.lock` 并 fail-closed；
该文件锁不能跨 Orin/RK 两块板，RK proxy 与 Orin Gateway 的互斥仍必须由部署
清单和启动编排保证。

默认型号为轮足 `zsl-1w`。点足型号可增加 `-DZSIBOT_MODEL=zsl-1`。默认 SDK 根目录为仓库根目录下的 `zsibot_sdk`，如 SDK 放在其他路径，可增加 `-DZSIBOT_SDK_ROOT=/absolute/path/to/zsibot_sdk`。

有两种双板控制方式：

1. 仅在隔离迁移测试中，若要尽量少改 RK3588 配置，可使用旧 UDP proxy
方式。Orin NX 运行 ROS 2 UDP client，RK3588 运行一个轻量 proxy，本质上让
SDK 仍然在 RK 本机访问 `127.0.0.1:43988`：

```bash
# RK3588（旧链路必须显式确认）
cd rk_proxy
ENABLE_DEPRECATED_ZSIBOT_TRANSPORT=1 ./run_zsibot_sdk_proxy.sh

# Orin NX
ENABLE_DEPRECATED_ZSIBOT_TRANSPORT=1 ./run_cmd_udp_client_only.sh
```

真机 planner 联动启动：

```bash
ENABLE_DEPRECATED_ZSIBOT_TRANSPORT=1 ./run_real_planner_udp.sh
```

Orin 侧默认把 `/scan_planner/cmd_vel` 发送到 `192.168.234.1:44000`，配置文件是 `src/zsibot_cmd_bridge/config/zsibot_cmd_udp_client.yaml`。这种方式不需要修改 RK3588 的 `/opt/export/config/sdk_config.yaml`，但需要把打包产物里的 `rk_proxy/` 目录放到 RK3588 上并启动。

2. 原来的直接桥接方式是 Orin NX 直接运行 SDK client：

```bash
ros2 launch zsibot_cmd_bridge zsibot_cmd_bridge.launch.py \
  enable_deprecated_zsibot_transport:=true
```

或随真机规划一起启动：

```bash
ros2 launch scan_planner run.launch.py \
  is_real_world:=true controller_mode:=closed_loop \
  publish_robot_description:=false \
  enable_deprecated_zsibot_transport:=true use_zsibot_bridge:=true
```

桥接参数位于 `src/zsibot_cmd_bridge/config/zsibot_cmd_bridge.yaml`。其中 `local_ip` 是 Orin NX 在机器人控制网段的 IP，`dog_ip` 是 RK3588 运动控制板 IP。当前默认值适配 Orin NX `192.168.234.234`、RK3588 `192.168.234.1`。RK3588 侧还需要将 `/opt/export/config/sdk_config.yaml` 的 `target_ip` 配成 Orin NX 的 IP，`target_port` 与桥接节点 `local_port` 保持一致。

完整双板部署、冒烟测试和故障排查见 `doc/orin_zsibot_deployment_guide.md`、`tools/orin_runtime/README_ORIN_RUNTIME.md` 和 `src/zsibot_cmd_bridge/README.md`。

### Orin NX sysroot 交叉编译

如果已经从 Orin NX 下载了 sysroot，例如 `/home/user/jetson/orin-nx/sysroot`，可以在开发机上直接使用仓库里的交叉编译脚本：

```bash
cd /home/user/robot/SCAN-Planner
ORIN_NX_SYSROOT=/home/user/jetson/orin-nx/sysroot tools/cross/build_orin_nx.sh
```

该命令默认只构建 `scan_planner` 依赖链。临时兼容若只需要不持有厂商
SDK 的 UDP 客户端，使用 `BUILD_LEGACY_ZSIBOT_UDP_CLIENT=1`；只有隔离台架
才允许使用 `BUILD_DEPRECATED_ZSIBOT_SDK_TARGETS=1` 产出 direct bridge/proxy，
且绝不能与统一 Gateway 同时运行。

这套流程使用的是 sysroot 里的 `aarch64-linux-gnu-gcc/g++`。由于 sysroot 里的编译器是 ARM64 板端原生编译器，x86 开发机不能直接执行它，脚本会通过 `qemu-aarch64-static` wrapper 调用。实测在本机完成 `scan_planner` 和 `zsibot_cmd_bridge` 依赖链编译，输出目录为：

```bash
install-orin-sysroot/
```

可用下面命令确认产物架构：

```bash
file install-orin-sysroot/lib/scan_planner/scan_planner_node
# 仅 BUILD_LEGACY_ZSIBOT_UDP_CLIENT=1 时：
file install-orin-sysroot/lib/zsibot_cmd_bridge/zsibot_cmd_udp_client
```

正常应显示 `ELF 64-bit ... ARM aarch64`。

注意事项：

- 需要开发机已安装 `qemu-aarch64-static`。
- 脚本会修补 sysroot 中 ROS 2 CMake export 里的裸 `libpython3.10.so` 绝对路径，并保留 `.orin-cross-bak` 备份。
- 如果最终链接阶段报 `libblas.so.3`、`liblapack.so.3` 找不到，确认 sysroot 中存在 `/usr/lib/aarch64-linux-gnu/libblas.so.3` 和 `/usr/lib/aarch64-linux-gnu/liblapack.so.3`。
- qemu wrapper 编译速度比普通交叉编译慢，完整构建约数分钟到十几分钟。

## 配置与接口

规划器、控制器和仿真器的参数分别位于：
- `src/planner/plan_manage/config/planner.yaml`
- `src/planner/plan_manage/config/controllers.yaml`
- `src/planner/plan_manage/config/simulator.yaml`

ROS 2 参数名称使用点号分隔，例如 `grid_map.resolution`。预设路径点是由 xyz 三元组组成的浮点数组：

```yaml
scan_planner_node:
  ros__parameters:
    fsm.navi_mode: 2
    fsm.waypoints: [0.0, 0.0, 0.3, 5.0, 1.0, 0.3]
```

自定义消息类型为 `scan_planner_msgs/msg/Bspline` 和 `scan_planner_msgs/msg/DataDisp`。规划器相关话题均为相对话题，支持重映射，核心输出话题包括 `planning/bspline`、`planning/data_display` 和 `planning/go2_execution_frozen`。

关键点记录器现在是原生的 `rclpy` 可执行程序：

```bash
ros2 run scan_planner keypoint_recorder.py \
  --odom /state_estimation \
  --output keypoints.yaml
```

记录器会把当前 odom 位置保存为 `fsm.waypoints` 参数文件。上板使用 `navi_mode=2` 时传入：

```bash
ros2 launch scan_planner run.launch.py \
  is_real_world:=true \
  navi_mode:=2 \
  keypoints_file:=/absolute/path/to/keypoints.yaml \
  publish_robot_description:=false
```

waypoint 坐标是 `odom` 坐标系下的绝对坐标。

## Gazebo Fortress / Go2 仿真

Go2 四足机器人物理模型基于 Gazebo Fortress、`ros_gz_sim` 和 `gz_ros2_control` 构建，对外提供 12 关节的 `joint_trajectory_controller`、`/joint_states` 话题、IMU 数据、四个足端接触力话题以及 `/clock` 时钟话题：

```bash
ros2 launch go2_description go2_sim.launch.py
```

如需在不启动物理仿真时查看模型，可运行：

```bash
ros2 launch go2_description go2_rviz.launch.py
```
旧版 Gazebo Classic 的轨迹/力可视化插件、外力插件以及宇树 ROS 1 专用插件已不在本 ROS 2 仿真版本中提供。


## 致谢

首先感谢原项目 [SCAN-Planner](https://github.com/wuyi2121/SCAN-Planner) 的作者 Han Zheng、Zhe Chen、Yiwen Fu、Ming Yang 和 Tong Qin 开源其研究成果与实现。本仓库在保留原项目 Apache-2.0 许可证的前提下完成 ROS 2 移植，完整署名见 [NOTICE](NOTICE)。

SCAN-Planner 的实现借鉴了 EGO-Planner、ROG-Map、MARSIM、Mockamap 和 Leg-KILO 的算法思路与开源代码，真实机器人定位方案基于 Elevator-LIO / FAST-LIO2 实现。

## 许可证

本仓库遵循 [Apache License 2.0](LICENSE)。分发或派生本项目时，请保留 [NOTICE](NOTICE) 中的原项目署名与许可证信息。
