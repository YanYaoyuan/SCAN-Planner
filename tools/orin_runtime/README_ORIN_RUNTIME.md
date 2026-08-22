# SCAN-Planner Orin NX Runtime Package

这个包是给 Orin NX 运行用的 ARM64 ROS 2 install 产物。

> **当前真机请先阅读：** 早期章节中的 `/state_estimation*`、
> `/body_state_estimation*`、`lio_map`、`livox_frame` 和
> `lidar_to_body_odom` 仅用于旧 bag/旧 runtime。当前默认链路使用
> `/omni/tf_manager/body_odom_global`、`/cloud_registered_global`、
> `omni_map`、`omni_base_link`，并要求
> `/omni/tf_manager/ready=true`。
> 源码工作区直接运行时，请优先按
> [`../../doc/局部路径闭环控制升级与真机测试指南.md`](../../doc/局部路径闭环控制升级与真机测试指南.md)
> 操作。
>
> **旧控制链路只用于隔离迁移测试。** 产品运行时 RK3588 不启动
> `zsibot_sdk_proxy`，Orin 只运行统一 robot bridge。文件锁只能保护同一主机，
> 不能证明另一块板没有 SDK owner。

## 1. 解压

建议放到 Orin NX 的 `/app`：

```bash
mkdir -p /app
cd /app
tar -xzf scan_planner_orin_nx_aarch64_20260719.tar.gz
cd scan_planner_orin_nx_aarch64_20260719
```

## 2. 检查运行环境

Orin NX 上需要有 ROS 2 Humble，以及和 sysroot 对应的 PCL/OpenCV/cv_bridge 等运行库：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 pkg list | grep scan_planner
```

确认产物是 ARM64：

```bash
file install/lib/scan_planner/scan_planner_node
```

默认产品包不应包含 `zsibot_cmd_bridge` 或 `zsibot_sdk_proxy`。只有显式构建
UDP 兼容客户端时，才额外检查
`install/lib/zsibot_cmd_bridge/zsibot_cmd_udp_client`。

## 3. 先测试 Orin 到 RK3588 网络

按当前机器狗网络：

```bash
ping 192.168.234.1
ssh 192.168.234.1
```

Orin NX 的控制网 IP 默认按 `192.168.234.234` 配置，RK3588 默认按 `192.168.234.1` 配置。

## 4. 只启动旧控制桥接（仅隔离迁移测试）

如果你愿意修改 RK3588 的 `/opt/export/config/sdk_config.yaml`，可以用原来的直接桥接模式。先不要启动 planner，单独测试 `/scan_planner/cmd_vel` 到 SDK：

```bash
ENABLE_DEPRECATED_ZSIBOT_TRANSPORT=1 ./run_bridge_only.sh
```

另开终端发小速度：

```bash
source /opt/ros/humble/setup.bash
source /app/scan_planner_orin_nx_aarch64_20260719/install/setup.bash
ros2 topic pub --rate 20 /scan_planner/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.05, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

新版本的运行脚本默认让 bridge 订阅 `/scan_planner/cmd_vel`，这是为了避免和机器狗已有的 `ecal2ros2` `/cmd_vel` 通道同时控制底盘。
按 `Ctrl-C` 停止发布后，bridge 会因为超时自动发送零速度。短时 `--once` 只能让机器狗往前动一下，不代表持续速度控制有问题。

确认机器狗方向无误后，再测试 `y` 和 `yaw`。当前桥接默认速度上限是 `vx=0.3 m/s`、`vy=0.15 m/s`、`yaw=0.5 rad/s`。

### 4.1 不修改 RK3588 配置的 UDP proxy 模式

如果不想改 RK3588 的 `/opt/export/config/sdk_config.yaml`，用这一套：

```text
Orin /scan_planner/cmd_vel
  -> zsibot_cmd_udp_client
  -> UDP 192.168.234.1:44000
  -> RK zsibot_sdk_proxy
  -> SDK 127.0.0.1:43988 -> 127.0.0.1
```

先把包里的 `rk_proxy/` 目录放到 RK3588，例如 `/app/rk_proxy`。在 RK3588 上启动：

```bash
cd /app/rk_proxy
ENABLE_DEPRECATED_ZSIBOT_TRANSPORT=1 ./run_zsibot_sdk_proxy.sh
```

正常应看到：

```text
ZsiBot SDK proxy listening on 0.0.0.0:44000
Connecting SDK locally: 127.0.0.1:43988 -> 127.0.0.1
standUp() returned 0
Waiting for standUp state before sending move commands
standUp state confirmed: ctrlmode=1; move commands enabled
```

然后在 Orin NX 上单独启动 UDP client：

```bash
cd /app/scan_planner_orin_nx_aarch64_20260719
ENABLE_DEPRECATED_ZSIBOT_TRANSPORT=1 ./run_cmd_udp_client_only.sh
```

另开 Orin 终端发小速度：

```bash
source /opt/ros/humble/setup.bash
source /app/scan_planner_orin_nx_aarch64_20260719/install/setup.bash
ros2 topic pub --rate 20 /scan_planner/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.05, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

停止命令：

```bash
ros2 topic pub --once /scan_planner/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

只要 RK proxy 日志里 `connected=true` 且能收到 `seq` 递增，就说明 Orin 到 RK 的 UDP 控制链路通了。如果 RK proxy 一直 `connected=false battery=0 ctrlmode=0`，问题在 RK 本机 SDK 到运动控制程序这一段，先在 RK 上跑官方 highlevel demo 验证默认 `127.0.0.1` 配置是否可用。

## 5. 启动 FAST_LIO 后运行 planner

确认 FAST_LIO 已发布：

```bash
ros2 topic echo /state_estimation --once
ros2 topic echo /cloud_registered --once
```

然后启动真机 planner：

```bash
./run_real_planner.sh
```

该通用脚本默认只启动规划和控制器，由统一 robot bridge 消费
`/scan_planner/cmd_vel`，不会启动旧 `zsibot_cmd_bridge`。仅在临时迁移测试时
使用 `ENABLE_DEPRECATED_ZSIBOT_TRANSPORT=1 ./run_real_planner.sh`。

如果采用“不修改 RK 配置”的 UDP proxy 模式，先保持 RK 上的 `run_zsibot_sdk_proxy.sh` 运行，然后在 Orin 上用：

```bash
ENABLE_DEPRECATED_ZSIBOT_TRANSPORT=1 ./run_real_planner_udp.sh
```

当前两个真机启动脚本使用的主要映射：

- `publish_robot_description:=false`
- `real_cmd_vel_topic:=/scan_planner/cmd_vel`
- `real_body_pose_topic:=/omni/tf_manager/body_odom_global`
- `real_cloud_topic:=/cloud_registered_global`
- `real_grid_frame_id:=omni_map`
- `goal_frame_id:=omni_map`
- `body_frame_id:=omni_base_link`
- `sensor_frame_id:=omni_lidar_link`
- `require_tf_ready:=true`
- `tf_ready_topic:=/omni/tf_manager/ready`
- `real_cloud_is_world:=true`
- `real_need_extrinsic:=false`
- `run_real_planner.sh`：默认两个旧 transport 均为 `false`
- `run_real_planner_udp.sh`：显式确认后才设置 `use_zsibot_udp_client:=true`

如果新狗上 LIO、TF、点云话题不同，优先改 `run_real_planner.sh` 里的这些 launch 参数。
使用 UDP proxy 模式时则改 `run_real_planner_udp.sh`。两个脚本也支持环境变量覆盖，例如：

```bash
ENABLE_DEPRECATED_ZSIBOT_TRANSPORT=1 \
GRID_FRAME_ID=omni_map \
BODY_FRAME_ID=omni_base_link \
SENSOR_FRAME_ID=omni_lidar_link \
./run_real_planner_udp.sh
```

真机默认使用更像路径跟踪 demo 的纯追踪输出：`controller_tracking_mode=path_follow`
和 `controller_drive_mode=pure_pursuit`。这时控制器只输出 `linear.x` 和
`angular.z`，`linear.y` 强制为 0，避免把机器狗当作稳定全向底盘。现场可以这样低速调：

```bash
ENABLE_DEPRECATED_ZSIBOT_TRANSPORT=1 \
CONTROLLER_PURE_PURSUIT_SPEED=0.15 \
CONTROLLER_LOOKAHEAD_DIST=0.50 \
CONTROLLER_MAX_VYAW=0.5 \
./run_real_planner_udp.sh
```

如果要和旧的全向输出对比，临时加：

```bash
ENABLE_DEPRECATED_ZSIBOT_TRANSPORT=1 \
CONTROLLER_DRIVE_MODE=omni ./run_real_planner_udp.sh
```

### 5.1 真机 TF 必须先整理

所有生产 TF 由 `omni_tf_manager` 唯一管理。FAST_LIO 必须关闭 TF 发布，并使用
profile 规定的 frame；planner 不再创建激光到机身的适配 TF：

```yaml
common:
  map_frame_id: "omni_map"
  odom_frame_id: "omni_odom"
  sensor_frame_id: "omni_imu_link"
  lidar_frame_id: "omni_lidar_link"
publish:
  tf_en: false
```

启动 planner 前必须验证 Manager 已就绪：

```bash
ros2 topic echo /omni/tf_manager/ready --qos-durability transient_local --once
ros2 run tf2_ros tf2_echo omni_map omni_base_link
```

`ready=false` 时 FSM 会清除当前轨迹并回到等待状态；不要绕过该门控发送导航目标。

### 5.2 使用 Manager 输出的机身位姿

机身位姿由 `omni_tf_manager` 根据审核过的 6DoF 外参生成。SCAN-Planner 只订阅
`/omni/tf_manager/body_odom_global`，不再接受或配置 `body_to_sensor_*`：

```bash
ros2 launch scan_planner run.launch.py \
  is_real_world:=true \
  sensor_type:=lidar \
  controller_mode:=closed_loop \
  publish_robot_description:=false \
  enable_deprecated_zsibot_transport:=true \
  use_zsibot_udp_client:=true \
  body_frame_id:=omni_base_link \
  sensor_frame_id:=omni_lidar_link \
  require_tf_ready:=true \
  tf_ready_topic:=/omni/tf_manager/ready \
  real_body_pose_topic:=/omni/tf_manager/body_odom_global \
  real_cloud_topic:=/cloud_registered_global \
  real_grid_frame_id:=omni_map \
  goal_frame_id:=omni_map \
  real_cmd_vel_topic:=/scan_planner/cmd_vel
```

验证：

```bash
ros2 topic echo /omni/tf_manager/body_odom_global --once
ros2 topic info /scan_planner/cmd_vel
ros2 topic info /cmd_vel
```

期望是 planner/bridge 只走 `/scan_planner/cmd_vel`；`/cmd_vel` 不再同时被 planner 和 `ecal2ros2` 控制。

## 6. 录制并运行 waypoint 路线

确认 `/omni/tf_manager/body_odom_global` 正常后，启动记录器：

```bash
cd /app/scan_planner_orin_nx_aarch64_20260719
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run scan_planner keypoint_recorder.py \
  --odom /omni/tf_manager/body_odom_global \
  --output /app/scan_planner_orin_nx_aarch64_20260719/keypoints.yaml
```

按键：

```text
a / Space / Enter  添加当前点
l                  查看点列表
u                  撤销最后一个点
s                  保存
q                  保存并退出
```

这些 waypoint 是 `omni_map` 坐标系下的绝对坐标。

使用录好的点运行 `navi_mode=2`：

```bash
ros2 launch scan_planner run.launch.py \
  is_real_world:=true \
  navi_mode:=2 \
  keypoints_file:=/app/scan_planner_orin_nx_aarch64_20260719/keypoints.yaml \
  sensor_type:=lidar \
  controller_mode:=closed_loop \
  publish_robot_description:=false \
  enable_deprecated_zsibot_transport:=true \
  use_zsibot_bridge:=true \
  body_frame_id:=omni_base_link \
  sensor_frame_id:=omni_lidar_link \
  require_tf_ready:=true \
  tf_ready_topic:=/omni/tf_manager/ready \
  real_body_pose_topic:=/omni/tf_manager/body_odom_global \
  real_cloud_topic:=/cloud_registered_global \
  real_grid_frame_id:=omni_map \
  real_cloud_is_world:=true \
  real_need_extrinsic:=false \
  goal_frame_id:=omni_map \
  real_cmd_vel_topic:=/scan_planner/cmd_vel
```

如果使用 UDP proxy 模式，把上面命令里的 `use_zsibot_bridge:=true` 改成：

```bash
use_zsibot_udp_client:=true
```

同时保留 `enable_deprecated_zsibot_transport:=true`；它是旧 direct/UDP 两种
兼容模式共同的二次确认开关。
# 当前真机默认接口（Omni TF authority）

> **请优先按本节以及真机测试指南操作。** 本节之前出现的
> `/state_estimation*`、`/body_state_estimation*`、`lio_map` 和
> `livox_frame` 命令是旧链路调试记录，不适用于当前真机默认配置。

本文件后部包含早期 `lio_odom` 调试记录。当前两个 `run_real_planner*.sh` 已统一为：

```text
BODY_ODOM_TOPIC=/omni/tf_manager/body_odom_global
TF_READY_TOPIC=/omni/tf_manager/ready
GRID_FRAME_ID=omni_map
BODY_FRAME_ID=omni_base_link
SENSOR_FRAME_ID=omni_lidar_link
real_cloud_topic=/cloud_registered_global
navi_mode=3
use_global_path_publisher=true
```

完整链路和验收项见
[`../../doc/SCAN_Planner_修改清单.md`](../../doc/SCAN_Planner_修改清单.md)。
旧 bag 若只包含本地话题，仍可使用后文或 `tools/offline_validation` 的
`lio_odom` 回放方式，但不要与当前真机全局链路混用。
