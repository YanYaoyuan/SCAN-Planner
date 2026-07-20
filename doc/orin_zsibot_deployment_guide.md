# SCAN-Planner + ZsiBot Orin NX 部署手册

本文档面向当前双板机器狗：

- Orin NX：运行 SCAN-Planner、LIO/感知、`zsibot_cmd_bridge` 或 `zsibot_cmd_udp_client`
- RK3588：运行 ZsiBot 原厂运动控制程序；UDP proxy 模式下额外运行 `zsibot_sdk_proxy`
- Orin NX IP：`192.168.234.234`
- RK3588 IP：`192.168.234.1`
- SDK 端口：`43988`
- 默认型号：`zsl-1w`
- 默认 LIO：FAST_LIO `omni_dog` / `omni_dog_relocalization`

## 1. 总体链路

```text
LIO / Sensor Driver
  -> odom / sensor_pose / cloud / depth

SCAN-Planner on Orin NX
  -> /cmd_vel

方案 A，不修改 RK 配置：
zsibot_cmd_udp_client on Orin NX
  -> UDP 192.168.234.1:44000
zsibot_sdk_proxy on RK3588
  -> SDK 127.0.0.1:43988 -> 127.0.0.1
  -> ZsiBot HighLevel::move(vx, vy, yaw_rate)

方案 B，直接桥接：
zsibot_cmd_bridge on Orin NX
  -> ZsiBot HighLevel::move(vx, vy, yaw_rate)

RK3588 motion_control
  -> motors / gait controller
```

`/cmd_vel` 是最终控制机器狗的速度命令。`/planning/bspline` 是规划器内部轨迹输出，不能直接接机器狗底层。

## 2. Clone 代码

在 Orin NX 上执行：

```bash
cd ~
git clone git@github.com:YanYaoyuan/SCAN-Planner.git
cd SCAN-Planner
git checkout ros2-community
```

如果 Orin 上没有配置 GitHub SSH key，也可以使用 HTTPS 地址 clone。

## 3. Orin NX 环境依赖

确认系统是 Ubuntu 22.04 + ROS 2 Humble：

```bash
lsb_release -a
source /opt/ros/humble/setup.bash
ros2 --version
```

安装基础依赖：

```bash
sudo apt update
sudo apt install -y \
  build-essential cmake git python3-colcon-common-extensions python3-rosdep \
  libeigen3-dev libpcl-dev libopencv-dev libarmadillo-dev libboost-all-dev \
  ros-humble-cv-bridge ros-humble-pcl-conversions \
  ros-humble-tf2-ros ros-humble-tf2-geometry-msgs \
  ros-humble-robot-state-publisher ros-humble-xacro
```

如果 `rosdep` 可用，也可以执行：

```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

## 4. RK3588 SDK 通信方案

从 Orin NX 登录 RK3588：

```bash
ssh firefly@192.168.234.1
```

### 4.1 推荐：不修改 RK3588 配置，使用 UDP proxy

这种方式保留 RK3588 默认 SDK 配置：

```yaml
target_ip: "127.0.0.1"
target_port: 43988
```

需要把打包产物中的 `rk_proxy/` 目录放到 RK3588，比如 `/app/rk_proxy`，然后在 RK3588 上运行：

```bash
cd /app/rk_proxy
./run_zsibot_sdk_proxy.sh
```

它会在 RK 上监听 `0.0.0.0:44000`，收到 Orin 的 UDP 速度包后，在 RK 本机调用 SDK：

```text
Orin /cmd_vel -> UDP 192.168.234.1:44000 -> RK proxy -> SDK 127.0.0.1:43988
```

Orin 侧使用 `zsibot_cmd_udp_client` 或 `run_real_planner_udp.sh`。这条链路不需要改 `/opt/export/config/sdk_config.yaml`，但 RK 上必须保持 `zsibot_sdk_proxy` 进程运行。

### 4.2 备选：直接桥接，需要修改 RK3588 配置

如果不想在 RK 上多跑 proxy，也可以让 Orin 直接运行 SDK client。此时要修改 RK3588 上的 SDK 配置：

```bash
sudo vim /opt/export/config/sdk_config.yaml
```

内容应为：

```yaml
target_ip: "192.168.234.234"
target_port: 43988
```

改完后重启运动控制程序或直接重启 RK3588/整机，让配置生效。

当前使用默认 `192.168.234.x` 网段，通常不需要改 `SDK_CLIENT_IP`。如果未来改成有线 `192.168.168.x` 或其他网段，再按 ZsiBot SDK 文档配置 `SDK_CLIENT_IP`。

## 5. Orin NX 编译

在 Orin NX 上：

```bash
cd ~/SCAN-Planner
source /opt/ros/humble/setup.bash

colcon build --symlink-install \
  --packages-select \
  scan_planner_msgs plan_env path_searching bspline_opt traj_utils \
  go2_description scan_planner zsibot_cmd_bridge \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

说明：

- 这是真机最小编译集合，不会编译 `local_sensing_node`、`mockamap`、`map_generator` 等仿真包。
- 默认编译 `zsl-1w`。
- 如果以后换成点足版本，增加 `-DZSIBOT_MODEL=zsl-1`。

不要在 Orin 真机部署时使用 `--packages-up-to scan_planner`，它会把仿真相关运行依赖也拉进来，可能因为 `glm`、Gazebo、RViz 等桌面依赖失败。

编译成功后：

```bash
source install/setup.bash
```

建议把 source 写进当前终端会话，不建议一开始就写进 `.bashrc`，等链路跑通后再固化。

## 6. 启动前检查

### 6.1 网络

在 Orin NX 上：

```bash
ip addr
ping 192.168.234.1
ssh firefly@192.168.234.1
```

确认 Orin 的机器人网段 IP 是 `192.168.234.234`。

### 6.2 SDK 动态库

编译后检查 bridge 可执行文件是否能找到库：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ldd install/zsibot_cmd_bridge/lib/zsibot_cmd_bridge/zsibot_cmd_bridge
```

如果出现 `libmc_sdk_zsl_1w_aarch64.so => not found`，通常是没有正确 source `install/setup.bash`，或 SDK 库没有被安装到 install 目录。

### 6.3 ROS 话题

启动你的 LIO、雷达、相机驱动后，确认真实话题名：

```bash
ros2 topic list
ros2 topic info /你的/odom
ros2 topic info /你的/points
ros2 topic hz /你的/odom
ros2 topic hz /你的/points
```

Planner 需要：

| 数据 | 推荐类型 |
| --- | --- |
| 机身里程计 | `nav_msgs/Odometry` |
| 传感器位姿 | `nav_msgs/Odometry` |
| 雷达点云 | `sensor_msgs/PointCloud2` |
| 深度图 | `sensor_msgs/Image` |

## 7. 启动 Planner + Bridge

当前 SCAN-Planner 真机默认值已经适配 FAST_LIO 的 `omni_dog.launch.py`
和 `omni_dog_relocalization.launch.py`：

| SCAN-Planner launch 参数 | 默认值 | 来源 |
| --- | --- | --- |
| `real_body_pose_topic` | `/state_estimation` | FAST_LIO `/Odometry` remap |
| `real_sensor_pose_topic` | `/state_estimation` | FAST_LIO 里 `base_frame_id == sensor_frame_id == livox_frame` |
| `real_cloud_topic` | `/cloud_registered` | FAST_LIO 世界系点云 |
| `real_grid_frame_id` | `odom` | FAST_LIO `common.odom_frame_id` |
| `real_cloud_is_world` | `true` | `/cloud_registered` 已转到 odom/world 系 |
| `real_need_extrinsic` | `false` | 使用 FAST_LIO 发布的传感器位姿 |

因此 FAST_LIO 已启动后，推荐先按“不修改 RK 配置”的 UDP proxy 方式运行。

先在 RK3588 上保持 proxy 运行：

```bash
cd /app/rk_proxy
./run_zsibot_sdk_proxy.sh
```

再在 Orin NX 上启动 planner：

```bash
source /opt/ros/humble/setup.bash
source ~/SCAN-Planner/install/setup.bash

ros2 launch scan_planner run.launch.py \
  is_real_world:=true \
  sensor_type:=lidar \
  controller_mode:=closed_loop \
  publish_robot_description:=false \
  use_zsibot_udp_client:=true
```

如果现场 FAST_LIO 输出被改名，启动时传参：

```bash
ros2 launch scan_planner run.launch.py \
  is_real_world:=true \
  sensor_type:=lidar \
  controller_mode:=closed_loop \
  publish_robot_description:=false \
  use_zsibot_udp_client:=true \
  real_body_pose_topic:=/state_estimation \
  real_sensor_pose_topic:=/state_estimation \
  real_cloud_topic:=/cloud_registered \
  real_grid_frame_id:=odom \
  real_cloud_is_world:=true \
  real_need_extrinsic:=false \
  real_cmd_vel_topic:=/cmd_vel
```

深度相机模式：

```bash
ros2 launch scan_planner run.launch.py \
  is_real_world:=true \
  sensor_type:=depth \
  controller_mode:=closed_loop \
  publish_robot_description:=false \
  use_zsibot_udp_client:=true \
  real_body_pose_topic:=/your/robot/odom \
  real_sensor_pose_topic:=/your/camera/odom \
  real_depth_topic:=/your/depth/image \
  real_cmd_vel_topic:=/cmd_vel
```

如果使用“直接桥接”方案，则把上面命令里的 `use_zsibot_udp_client:=true` 改成 `use_zsibot_bridge:=true`，并确认 RK3588 的 `/opt/export/config/sdk_config.yaml` 已经把 `target_ip` 改成 Orin NX 的 IP。

### 7.1 录制 waypoint 并使用 navi_mode=2

`navi_mode=2` 使用配置文件里的 `fsm.waypoints`，适合先用遥控器/手动方式把机器狗带到几个关键点，然后保存成一条预设路线。

先确认 FAST_LIO 已经发布 odom：

```bash
ros2 topic echo /state_estimation --once
```

启动 waypoint 记录器：

```bash
cd /app/scan_planner_orin_nx_aarch64_20260719
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run scan_planner keypoint_recorder.py \
  --odom /state_estimation \
  --output /app/scan_planner_orin_nx_aarch64_20260719/keypoints.yaml
```

记录器按键：

```text
Enter / Space / a  记录当前机器狗位置为 waypoint
l                  列出已记录 waypoint
u                  撤销最后一个 waypoint
r                  替换指定 waypoint
d                  删除指定 waypoint
s                  保存
q                  保存并退出
```

保存后的 `keypoints.yaml` 类似：

```yaml
scan_planner_node:
  ros__parameters:
    fsm.waypoints: [0.5, 0, 0.3, 1.0, 0, 0.3]
```

这些点是 `odom` 坐标系下的绝对坐标，不是相对移动量。记录点时脚本直接读取 `/state_estimation.pose.pose.position`。

用录好的 waypoint 跑预设路线：

```bash
ros2 launch scan_planner run.launch.py \
  is_real_world:=true \
  navi_mode:=2 \
  keypoints_file:=/app/scan_planner_orin_nx_aarch64_20260719/keypoints.yaml \
  sensor_type:=lidar \
  controller_mode:=closed_loop \
  publish_robot_description:=false \
  use_zsibot_udp_client:=true \
  real_body_pose_topic:=/state_estimation \
  real_sensor_pose_topic:=/state_estimation \
  real_cloud_topic:=/cloud_registered \
  real_grid_frame_id:=odom \
  real_cloud_is_world:=true \
  real_need_extrinsic:=false
```

启动后，planner 收到第一帧 `/state_estimation` 就会开始规划到第一个 waypoint。接近当前 waypoint 约 `0.5m`，或者当前段轨迹执行结束后，会自动切到下一个 waypoint。

## 8. 需要重点适配的参数

Planner 参数在：

```text
src/planner/plan_manage/config/planner.yaml
```

重点检查：

```yaml
grid_map.frame_id: world
grid_map.sensor_type: lidar
grid_map.cloud_is_world: true
grid_map.need_extrinsic: false
grid_map.cx: ...
grid_map.cy: ...
grid_map.fx: ...
grid_map.fy: ...
grid_map.k_depth_scaling_factor: 1000.0
```

使用当前 FAST_LIO 配置时，launch 会覆盖以下参数：

```yaml
grid_map.frame_id: odom
grid_map.cloud_is_world: true
grid_map.need_extrinsic: false
```

### 8.1 frame_id

`grid_map.frame_id` 应该和你的里程计世界系一致，例如 `odom`、`map` 或 `world`。

如果你的 odom 消息 header 是 `odom`，建议改：

```yaml
grid_map.frame_id: odom
```

### 8.2 点云坐标系

如果雷达点云已经是世界系点云：

```yaml
grid_map.cloud_is_world: true
grid_map.need_extrinsic: false
```

如果雷达点云是雷达自身坐标系，且 `real_sensor_pose_topic` 发布的是雷达在世界系下的位姿：

```yaml
grid_map.cloud_is_world: false
grid_map.need_extrinsic: false
```

如果 `real_sensor_pose_topic` 发布的是机身位姿，传感器相对机身还有外参：

```yaml
grid_map.cloud_is_world: false
grid_map.need_extrinsic: true
```

这时需要确认 `src/planner/plan_env/src/grid_map.cpp` 里的 `lidar_extrinsic_` 或 `depth_extrinsic_` 是否匹配新狗实际安装位置。更推荐写一个小节点直接发布“传感器真实世界位姿”，然后把 `need_extrinsic` 设为 `false`，现场更不容易乱。

### 8.3 深度相机内参

使用 `sensor_type:=depth` 时，必须确认：

```yaml
grid_map.cx
grid_map.cy
grid_map.fx
grid_map.fy
grid_map.k_depth_scaling_factor
```

常见情况：

- 16UC1 深度图，单位毫米：`k_depth_scaling_factor: 1000.0`
- 32FC1 深度图，单位米：代码会转换，但仍建议实测确认

## 9. 单独测试 ZsiBot 控制链路

### 9.1 UDP proxy 模式

先在 RK3588 上启动 proxy：

```bash
cd /app/rk_proxy
./run_zsibot_sdk_proxy.sh
```

再在 Orin NX 上只启动 UDP client：

```bash
source /opt/ros/humble/setup.bash
source ~/SCAN-Planner/install/setup.bash

ros2 launch zsibot_cmd_bridge zsibot_cmd_udp_client.launch.py
```

另一个 Orin 终端发送小速度：

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.05, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

RK proxy 日志里 `seq` 递增、`connected=true`，并且机器狗能站立/低速动/停住，就说明链路通了。

### 9.2 直接 bridge 模式

不启动 planner，只测试 Orin 上的 SDK bridge：

```bash
source /opt/ros/humble/setup.bash
source ~/SCAN-Planner/install/setup.bash

ros2 launch zsibot_cmd_bridge zsibot_cmd_bridge.launch.py
```

另一个终端发送小速度：

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

停止：

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

Bridge 和 UDP proxy 默认 50Hz 重发最近速度，`cmd_timeout` 后会发零速。

正常日志类似：

```text
SDK status: connected=true battery=85% ctrlmode=1 cmd=active vx=0.100 vy=0.000 yaw=0.000
```

控制模式含义：

| ctrlmode | 含义 |
| --- | --- |
| `0` | 阻尼模式 |
| `1` | 站立模式 |
| `3` | 移动模式 |

## 10. 常见问题排查

### 10.1 编译找不到 SDK 库

现象：

```text
Could not find mc_sdk_zsl_1w_aarch64
```

检查：

```bash
ls zsibot_sdk/lib/zsl-1w/aarch64
```

Orin NX 应该使用：

```text
libmc_sdk_zsl_1w_aarch64.so
```

如果目录不存在，说明 SDK 没 clone 完整或分支不对。

### 10.2 启动时报 ROS 包找不到

现象：

```text
Package 'scan_planner' not found
Package 'zsibot_cmd_bridge' not found
```

处理：

```bash
source /opt/ros/humble/setup.bash
source ~/SCAN-Planner/install/setup.bash
ros2 pkg list | grep zsibot
```

如果仍没有，重新编译。

### 10.3 控制链路 connected=false

重点检查：

```bash
ping 192.168.234.1
ssh firefly@192.168.234.1
```

如果用 UDP proxy 模式，先看 RK proxy 日志。如果 RK proxy 一直是：

```text
SDK status: connected=false battery=0% ctrlmode=0
```

说明 RK 本机 SDK 到运动控制程序没通，先在 RK 上跑官方 highlevel demo 验证默认 `127.0.0.1` SDK 配置。如果官方 demo 能跑，再检查 proxy 是否和官方 demo 同时占用 SDK。

如果用直接 bridge 模式，RK3588 上检查：

```bash
cat /opt/export/config/sdk_config.yaml
```

必须是：

```yaml
target_ip: "192.168.234.234"
target_port: 43988
```

改完要重启运动控制程序或整机。

### 10.4 move 返回非 0

可能原因：

- 机器狗还没站立
- 当前在阻尼/趴下状态
- SDK 被其他程序占用
- RK3588 运动控制程序异常

处理：

- 确认 `auto_stand: true`
- 确认 `require_standing_before_move: true`，bridge 会等 `ctrlmode=1` 或 `ctrlmode=3` 后才发 `move()`
- 看 bridge 日志里的 `ctrlmode`
- 停掉其他 SDK demo
- 重启 RK3588 运动控制程序

如果日志持续出现：

```text
Cannot transition to 'move' state: must transition to 'standUp' first.
```

说明 `standUp()` 命令虽然发送成功，但 SDK 状态机尚未确认进入站立状态。新版 bridge 启动后会先等待：

```text
Waiting for standUp state before sending move commands
standUp state confirmed: ctrlmode=1; move commands enabled
```

在看到 `move commands enabled` 之前，不要发布 `/cmd_vel` 做运动测试。

### 10.5 Planner 一直 no odom

说明 `scan_planner_node` 没收到 `body_pose`。

检查：

```bash
ros2 topic echo /your/robot/odom --once
ros2 topic info /your/robot/odom
```

确认启动参数：

```bash
real_body_pose_topic:=/your/robot/odom
```

消息类型必须是 `nav_msgs/Odometry`。如果新狗只有 PoseStamped，需要写转换节点转成 Odometry。

### 10.6 地图不更新 / no sensor_pose

说明 `GridMap` 没收到传感器位姿。

检查：

```bash
ros2 topic echo /your/lidar/odom --once
```

确认启动参数：

```bash
real_sensor_pose_topic:=/your/lidar/odom
```

如果新狗只有 TF，没有传感器 Odometry 话题，需要写 TF-to-Odometry 节点。

### 10.7 点云有数据但避障不对

检查顺序：

1. `grid_map.frame_id` 是否和 odom/map 坐标系一致。
2. `grid_map.cloud_is_world` 是否符合点云实际坐标系。
3. `grid_map.need_extrinsic` 和外参是否正确。
4. RViz 中看 `/grid_map/occupancy` 和 `/grid_map/occupancy_inflate` 是否落在机器人周围。

### 10.8 深度图模式没有障碍

检查：

```bash
ros2 topic info /your/depth/image
ros2 topic echo /your/camera/odom --once
```

确认：

- `sensor_type:=depth`
- `real_depth_topic` 正确
- `real_sensor_pose_topic` 正确
- `planner.yaml` 中相机内参正确
- 深度图编码和 `k_depth_scaling_factor` 匹配

### 10.9 机器狗方向不对

如果 `/cmd_vel.linear.x > 0` 时机器狗不是向前，常见原因是机身坐标系定义和 SDK 定义不同。

处理方式：

- 先单独测试 bridge，分别发 `x/y/yaw` 小速度。
- 如果方向反了，可以在 `zsibot_cmd_bridge` 里增加轴映射参数，或临时改桥接代码中的 `sendMove(vx, vy, yaw_rate)` 输入符号。
- 不建议直接改 planner 坐标系，先把底盘桥接层调成符合 ROS 常规：`x` 前、`y` 左、`yaw` 左转为正。

## 11. 推荐上板顺序

1. Orin 能 ping/ssh RK3588。
2. 优先使用 UDP proxy：RK3588 启动 `/app/rk_proxy/run_zsibot_sdk_proxy.sh`，不改 `sdk_config.yaml`。
3. 如果不用 proxy，再把 RK3588 `sdk_config.yaml` 配好并重启运动控制。
4. Orin 编译通过。
5. 启动 FAST_LIO，确认 `/state_estimation` 和 `/cloud_registered` 正常。
6. 单独启动 `zsibot_cmd_udp_client` 或 `zsibot_cmd_bridge`。
7. 手动发小 `/cmd_vel`，确认站立、前后、左右、旋转方向。
8. 启动 planner，但先不要给目标点，观察是否有 odom/map。
9. 给很近的目标点，低速测试。
10. 再逐步增大目标距离和速度限制。

## 12. 现场建议

- 第一次上板把 `manager.max_vel`、`closed_loop_controller.max_vx` 保持在 `0.3~0.5` 更稳。
- 保持急停/遥控器可用。
- 避免同时运行官方 SDK demo 和 `zsibot_cmd_bridge`。
- 每次改 RK3588 SDK 配置后都重启运动控制。
- 如果现场网络不稳定，先不要跑 planner，先用 bridge 冒烟测试。

## 13. 使用 Orin NX sysroot 交叉编译

如果已经从 Orin NX 下载了 sysroot，例如：

```bash
/home/user/jetson/orin-nx/sysroot
```

可以在开发机上用仓库里的 qemu wrapper 调用 sysroot 里的 ARM64 编译器：

```bash
cd /home/user/robot/SCAN-Planner
ORIN_NX_SYSROOT=/home/user/jetson/orin-nx/sysroot \
  tools/cross/build_orin_nx.sh
```

脚本会做三件事：

1. 修补 sysroot 中 ROS2 CMake export 里的 `libpython3.10.so` 绝对路径。
2. 使用 `tools/cross/orin_nx_toolchain.cmake` 调用 sysroot 里的 `aarch64-linux-gnu-gcc/g++`。
3. 编译 `scan_planner` 和 `zsibot_cmd_bridge` 需要的依赖链。

成功后产物在：

```bash
install-orin-sysroot/
```

可以用 `file` 确认是 ARM64：

```bash
file install-orin-sysroot/lib/scan_planner/scan_planner_node
file install-orin-sysroot/lib/zsibot_cmd_bridge/zsibot_cmd_bridge
```

正常应显示 `ELF 64-bit ... ARM aarch64`。

注意：sysroot 里的 `gcc/g++` 是 ARM64 板端原生编译器，不是 x86 可直接运行的交叉编译器；开发机上直接执行会报 `aarch64-binfmt-P: Could not open '/lib/ld-linux-aarch64.so.1'`。本仓库通过 `qemu-aarch64-static` wrapper 解决这个问题，因此编译速度会比普通交叉编译慢。
