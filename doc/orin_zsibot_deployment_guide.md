# SCAN-Planner + ZsiBot Orin NX 部署手册

> **Phase 0 下线提示（2026-08）：** 本文中的 `zsibot_cmd_bridge`、
> `zsibot_cmd_udp_client` 和 `zsibot_sdk_proxy` 均为旧兼容链路。产品部署应由
> 统一 robot bridge 独占厂商 SDK。旧链路默认禁用，只有隔离迁移测试才设置
> `ENABLE_DEPRECATED_ZSIBOT_TRANSPORT=1` 或 launch 参数
> `enable_deprecated_zsibot_transport:=true`，且两套 bridge 严禁同时运行。

> **TF 接口更新（2026-08）：** 当前生产链由 `omni_tf_manager` 独占全部
> 静态/动态 TF。Planner 只消费
> `/omni/tf_manager/body_odom_global`、`/cloud_registered_global` 和
> `/omni/tf_manager/ready`，坐标为 `omni_map`、`omni_base_link`、
> `omni_lidar_link`。本文后部出现的 `lidar_to_body_odom`、
> `body_to_sensor_*`、`lio_map`、`livox_frame` 命令均是历史记录，不得用于
> 当前真机。可执行命令以 `tools/orin_runtime/run_real_planner*.sh` 为准。

本文档面向当前双板机器狗。产品角色固定如下：

- Orin NX：运行 SCAN-Planner、LIO/感知和统一 `rosdeck_robot_bridge` Gateway；
- RK3588：只运行 ZsiBot 原厂运动控制程序，不运行 legacy `zsibot_sdk_proxy`；
- Orin NX IP：`192.168.234.234`
- RK3588 IP：`192.168.234.1`
- SDK 端口：`43988`
- 默认型号：`zsl-1w`
- 默认 LIO：FAST_LIO `omni_dog` / `omni_dog_relocalization`

## 1. 总体链路

```text
Sensor Driver -> omni_tf_manager -> canonical sensor topics / TF
omni_slam -> /omni/tf_manager/body_odom_global + /cloud_registered_global
omni_tf_manager -> /omni/tf_manager/ready

SCAN-Planner on Orin NX
  -> /scan_planner/cmd_vel

rosdeck_robot_bridge Gateway on Orin NX
  -> authority / E-stop / watchdog / velocity limits
  -> ZsiBot HighLevel::move(vx, vy, yaw_rate)

RK3588 motion_control
  -> motors / gait controller
```

`/scan_planner/cmd_vel` 是本工程默认使用的隔离速度命令，由统一 Gateway 消费。
`/planning/bspline` 是规划器内部轨迹输出，不能直接接机器狗底层；产品路径也不允许
planner 直接发布 SDK-facing final topic。

同机的 Gateway、旧 direct bridge 和旧 proxy 使用同一个
`/run/lock/omni/zsibot_sdk_owner.lock`，竞争时 fail-closed。但 `flock` 不能跨
Orin/RK 两块主机：上电编排必须另外确认 RK 上的 `zsibot_sdk_proxy` 和官方 SDK
demo 已停止。不能把“Orin 已拿到文件锁”当成跨板唯一 owner 的证明。

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

## 4. 旧 RK3588 SDK 通信方案（仅隔离迁移测试）

从 Orin NX 登录 RK3588：

```bash
ssh firefly@192.168.234.1
```

### 4.1 旧方案 A：不修改 RK3588 配置，使用 UDP proxy

这种方式保留 RK3588 默认 SDK 配置：

```yaml
target_ip: "127.0.0.1"
target_port: 43988
```

需要把打包产物中的 `rk_proxy/` 目录放到 RK3588，比如 `/app/rk_proxy`，然后在 RK3588 上运行：

```bash
cd /app/rk_proxy
ENABLE_DEPRECATED_ZSIBOT_TRANSPORT=1 ./run_zsibot_sdk_proxy.sh
```

它会在 RK 上监听 `0.0.0.0:44000`，收到 Orin 的 UDP 速度包后，在 RK 本机调用 SDK：

```text
Orin /scan_planner/cmd_vel -> UDP 192.168.234.1:44000 -> RK proxy -> SDK 127.0.0.1:43988
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
  --packages-up-to scan_planner \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

说明：

- 这是默认产品编译，不构建任何持有厂商 SDK 的旧 target。
- 隔离迁移测试若确实需要 direct bridge/proxy，额外选择
  `zsibot_cmd_bridge` 并增加
  `-DZSIBOT_ENABLE_DEPRECATED_SDK_TARGETS=ON`。
- 旧 SDK target 默认型号为 `zsl-1w`；点足版本增加 `-DZSIBOT_MODEL=zsl-1`。

`--packages-up-to scan_planner` 是当前默认产品构建入口，只构建规划器及其工作区
依赖；旧 ZsiBot transport 不在默认依赖闭包中。若要做临时 UDP 兼容包，应单独
选择 `zsibot_cmd_bridge`，并保持 `ZSIBOT_ENABLE_DEPRECATED_SDK_TARGETS=OFF`。

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

仅在隔离台架显式构建 direct bridge 后，才检查它能否找到 SDK 库：

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

### 6.4 真机 TF、时间和控制链路

发送 goal 前先确认这三件事，否则 Foxglove/RViz 里看起来能规划，实际控制会很危险。

第一，SLAM odom 和底盘 odom 不能都叫 `odom`。你当前 FAST_LIO 配置在：

```text
/home/user/robot/omni_slam/FAST_LIO/config/omni_dog.yaml
/home/user/robot/omni_slam/FAST_LIO/config/omni_dog_relocalization.yaml
```

关键项是：

```yaml
common:
  odom_frame_id: "odom"
  sensor_frame_id: "livox_frame"
  base_frame_id: "livox_frame"
  send_odom_base_tf: false
```

如果机器狗 `/robot_tf` 已经发布 `odom -> base_link`，建议把 FAST_LIO 的
`common.odom_frame_id` 改成 `lio_odom`，然后重启 SLAM。之后 SCAN-Planner
也用 `real_grid_frame_id:=lio_odom` 和 `goal_frame_id:=lio_odom`。

第二，FAST_LIO 的 `/state_estimation.child_frame_id` 是 `livox_frame`，它是激光，不是机身中心。新版本提供 `lidar_to_body_odom` 节点，可以从 `/state_estimation` 生成 `/body_state_estimation`。默认输出的 planner 机身坐标名是 `scan_base_link`，避免和机器狗原系统里的 `base_link` 冲突。参数 `body_to_sensor_*` 是 `scan_base_link -> livox_frame` 外参，单位是米和弧度；没量准外参前不要发导航 goal。

第三，控制链路只能保留一条。产品脚本默认让 planner 输出
`/scan_planner/cmd_vel`，再由统一 Gateway 订阅这个隔离话题。只有隔离迁移测试才
改由 `zsibot_cmd_bridge` 或 `zsibot_cmd_udp_client` 订阅；此时必须先停止产品
Gateway，并确认另一块板上也没有 SDK owner。

检查命令：

```bash
ros2 topic info /cmd_vel
ros2 topic info /scan_planner/cmd_vel
ros2 topic echo /state_estimation --once
ros2 topic echo /body_state_estimation --once
```

如果系统时间还是 `1970-01-01`，先修时间再看 TF：

```bash
timedatectl
sudo timedatectl set-ntp true
sudo hwclock --systohc
```

没有外网或没有电池 RTC 时，需要用板端可用的 NTP/GPS/PPS/RK 时间源做 systemd 启动同步；否则 ROS/TF 时间戳会停留在开机秒数，Foxglove 和 TF 缓存都可能异常。

## 7. 启动 Planner + 统一 Gateway

原始 SCAN-Planner 真机默认值可以直接接 FAST_LIO 的 `omni_dog.launch.py`
和 `omni_dog_relocalization.launch.py`，但它把激光位姿同时当作机身位姿，只适合不运动的联调：

| SCAN-Planner launch 参数 | 默认值 | 来源 |
| --- | --- | --- |
| `real_body_pose_topic` | `/state_estimation` | FAST_LIO `/Odometry` remap，不推荐真机闭环 |
| `real_sensor_pose_topic` | `/state_estimation` | FAST_LIO 里 `base_frame_id == sensor_frame_id == livox_frame` |
| `real_cloud_topic` | `/cloud_registered` | FAST_LIO 世界系点云 |
| `real_grid_frame_id` | `odom` | FAST_LIO `common.odom_frame_id` |
| `real_cloud_is_world` | `true` | `/cloud_registered` 已转到 odom/world 系 |
| `real_need_extrinsic` | `false` | 使用 FAST_LIO 发布的传感器位姿 |

真机闭环推荐使用“SLAM odom 隔离 + `/body_state_estimation` + `/scan_planner/cmd_vel`”方式运行。

产品启动前先确认 RK3588 上没有 legacy proxy：

```bash
ssh firefly@192.168.234.1 \
  "pgrep -af 'zsibot_sdk_proxy|highlevel.*demo' || true"
```

然后在 Orin NX 通过 systemd 启动统一产品服务；源码联调也必须使用
`product_bringup.launch.py`，不能直接执行 Gateway 单节点：

```bash
systemctl restart rosdeck-robot-bridge.service
# 或源码工作区：
ros2 launch rosdeck_robot_bridge product_bringup.launch.py
```

再启动 planner。下面示例假设已经把 FAST_LIO 的 `common.odom_frame_id` 改成了
`lio_odom`：

```bash
source /opt/ros/humble/setup.bash
source ~/SCAN-Planner/install/setup.bash

ros2 launch scan_planner run.launch.py \
  is_real_world:=true \
  sensor_type:=lidar \
  controller_mode:=closed_loop \
  publish_robot_description:=false \
  use_lidar_to_body_odom:=true \
  lidar_odom_topic:=/state_estimation \
  body_odom_topic:=/body_state_estimation \
  body_odom_frame_id:=scan_base_link \
  body_odom_sensor_frame_id:=livox_frame \
  body_odom_world_frame_id:=lio_odom \
  body_odom_publish_tf:=false \
  body_to_sensor_x:=0.0 \
  body_to_sensor_y:=0.0 \
  body_to_sensor_z:=0.0 \
  body_to_sensor_roll:=0.0 \
  body_to_sensor_pitch:=0.0 \
  body_to_sensor_yaw:=0.0 \
  real_sensor_pose_topic:=/your/state_estimation \
  real_cloud_topic:=/cloud_registered \
  real_grid_frame_id:=lio_odom \
  real_cloud_is_world:=true \
  real_need_extrinsic:=false \
  goal_frame_id:=lio_odom \
  real_cmd_vel_topic:=/scan_planner/cmd_vel
```

把 `body_to_sensor_*` 的 0 改成真实 `scan_base_link -> livox_frame` 外参。`body_odom_publish_tf:=false` 是故意的：它只给 planner 发布 odometry，不再发布一条新的 `lio_odom -> scan_base_link` TF，避免和机器狗自带 TF 混在一起。

如果现场 FAST_LIO 输出被改名，启动时传参：

```bash
ros2 launch scan_planner run.launch.py \
  is_real_world:=true \
  sensor_type:=lidar \
  controller_mode:=closed_loop \
  publish_robot_description:=false \
  use_lidar_to_body_odom:=true \
  lidar_odom_topic:=/your/state_estimation \
  body_odom_topic:=/body_state_estimation \
  body_odom_frame_id:=scan_base_link \
  body_odom_sensor_frame_id:=livox_frame \
  body_odom_world_frame_id:=lio_odom \
  body_odom_publish_tf:=false \
  real_sensor_pose_topic:=/state_estimation \
  real_cloud_topic:=/your/cloud_registered \
  real_grid_frame_id:=lio_odom \
  real_cloud_is_world:=true \
  real_need_extrinsic:=false \
  goal_frame_id:=lio_odom \
  real_cmd_vel_topic:=/scan_planner/cmd_vel
```

深度相机模式：

```bash
ros2 launch scan_planner run.launch.py \
  is_real_world:=true \
  sensor_type:=depth \
  controller_mode:=closed_loop \
  publish_robot_description:=false \
  real_body_pose_topic:=/your/robot/odom \
  real_sensor_pose_topic:=/your/camera/odom \
  real_depth_topic:=/your/depth/image \
  real_cmd_vel_topic:=/scan_planner/cmd_vel
```

若必须复现旧链路，只能在隔离台架先停止
`rosdeck-robot-bridge.service`，再按第 4、9 节显式启用 direct/proxy。旧链路参数
不得复制回上述产品命令。

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
  --odom /body_state_estimation \
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
omni_scan_planner:
  ros__parameters:
    fsm.waypoints: [0.5, 0, 0.3, 1.0, 0, 0.3]
```

这些点是 `lio_odom` 坐标系下的绝对坐标，不是相对移动量。记录点时脚本直接读取 `/body_state_estimation.pose.pose.position`。

用录好的 waypoint 跑预设路线：

```bash
ros2 launch scan_planner run.launch.py \
  is_real_world:=true \
  navi_mode:=2 \
  keypoints_file:=/app/scan_planner_orin_nx_aarch64_20260719/keypoints.yaml \
  sensor_type:=lidar \
  controller_mode:=closed_loop \
  publish_robot_description:=false \
  use_lidar_to_body_odom:=true \
  lidar_odom_topic:=/state_estimation \
  body_odom_topic:=/body_state_estimation \
  body_odom_frame_id:=scan_base_link \
  body_odom_sensor_frame_id:=livox_frame \
  body_odom_world_frame_id:=lio_odom \
  body_odom_publish_tf:=false \
  body_to_sensor_x:=0.0 \
  body_to_sensor_y:=0.0 \
  body_to_sensor_z:=0.0 \
  body_to_sensor_roll:=0.0 \
  body_to_sensor_pitch:=0.0 \
  body_to_sensor_yaw:=0.0 \
  real_sensor_pose_topic:=/state_estimation \
  real_cloud_topic:=/cloud_registered \
  real_grid_frame_id:=lio_odom \
  real_cloud_is_world:=true \
  real_need_extrinsic:=false \
  goal_frame_id:=lio_odom \
  real_cmd_vel_topic:=/scan_planner/cmd_vel
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

## 9. 单独测试旧 ZsiBot 控制链路（仅隔离迁移测试）

### 9.1 UDP proxy 模式

先在 RK3588 上启动 proxy：

```bash
cd /app/rk_proxy
ENABLE_DEPRECATED_ZSIBOT_TRANSPORT=1 ./run_zsibot_sdk_proxy.sh
```

再在 Orin NX 上只启动 UDP client：

```bash
source /opt/ros/humble/setup.bash
source ~/SCAN-Planner/install/setup.bash

ros2 launch zsibot_cmd_bridge zsibot_cmd_udp_client.launch.py \
  enable_deprecated_zsibot_transport:=true \
  cmd_vel_topic:=/scan_planner/cmd_vel
```

另一个 Orin 终端发送小速度：

```bash
ros2 topic pub --rate 20 /scan_planner/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.05, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

按 `Ctrl-C` 停止发布后，client 会因为超时自动发送零速度。RK proxy 日志里 `seq` 递增、`connected=true`，并且机器狗能站立/低速动/停住，就说明链路通了。

### 9.2 直接 bridge 模式

不启动 planner，只测试 Orin 上的 SDK bridge：

```bash
source /opt/ros/humble/setup.bash
source ~/SCAN-Planner/install/setup.bash

ros2 launch zsibot_cmd_bridge zsibot_cmd_bridge.launch.py \
  enable_deprecated_zsibot_transport:=true \
  cmd_vel_topic:=/scan_planner/cmd_vel
```

另一个终端发送小速度：

```bash
ros2 topic pub --rate 20 /scan_planner/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

停止：

```bash
ros2 topic pub --once /scan_planner/cmd_vel geometry_msgs/msg/Twist \
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
invalid velocity in x-axi, expect -3.7~-0.05/0.05~3.7m/s
invalid velocity in y-axi, expect -1~-0.1/0.1~1.0m/s
```

说明 SDK 收到了非零但低于最小有效值的速度。`zsl-1w` 的 `move()`
要求小速度要么传 0，要么超过最小门槛。bridge 配置里应保持：

```yaml
deadband_vx: 0.05
deadband_vy: 0.10
deadband_yaw_rate: 0.10
```

这样 planner 输出 `vy=0.01` 这类细小修正时会被转成 0，不会导致整条
`move(vx, vy, yaw_rate)` 被 SDK 拒绝。

如果日志持续出现：

```text
Cannot transition to 'move' state: must transition to 'standUp' first.
```

说明 `standUp()` 命令虽然发送成功，但 SDK 状态机尚未确认进入站立状态。新版 bridge 启动后会先等待：

```text
Waiting for standUp state before sending move commands
standUp state confirmed: ctrlmode=1; move commands enabled
```

在看到 `move commands enabled` 之前，不要发布 `/scan_planner/cmd_vel` 做运动测试。

### 10.5 Planner 一直 no odom

说明 `omni_scan_planner` 没收到 manager 输出或 TF authority 尚未 ready。检查：

```bash
ros2 topic echo /omni/tf_manager/ready --qos-durability transient_local --once
ros2 topic echo /omni/tf_manager/body_odom_global --once
ros2 run tf2_ros tf2_echo omni_map omni_base_link
```

确认启动参数：

```bash
require_tf_ready:=true
real_body_pose_topic:=/omni/tf_manager/body_odom_global
real_grid_frame_id:=omni_map
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

如果 `/scan_planner/cmd_vel.linear.x > 0` 时机器狗不是向前，常见原因是机身坐标系定义和 SDK 定义不同。

处理方式：

- 先单独测试 bridge，分别发 `x/y/yaw` 小速度。
- 如果方向反了，可以在 `zsibot_cmd_bridge` 里增加轴映射参数，或临时改桥接代码中的 `sendMove(vx, vy, yaw_rate)` 输入符号。
- 不建议直接改 planner 坐标系，先把底盘桥接层调成符合 ROS 常规：`x` 前、`y` 左、`yaw` 左转为正。

## 11. 产品上板顺序

1. Orin 能 ping/ssh RK3588。
2. RK3588 停止并禁用 `zsibot_sdk_proxy`、官方 SDK demo 等旧 owner。
3. Orin 产品包确认不包含 `zsibot_cmd_bridge`/`zsibot_sdk_proxy` 产物。
4. 启动 `rosdeck-robot-bridge.service`，确认 Gateway 与 safety supervisor 都存在。
5. 启动 FAST_LIO，确认 `/state_estimation` 和 `/cloud_registered` 正常。
6. 启动 planner，但先不 arm supervisor、不给目标点，观察 odom/map 与仲裁状态。
7. 现场确认后 arm supervisor，再显式 reset Gateway E-stop。
8. 获取 navigation 控制权后发很近的目标点，低速验证方向和急停。
9. 任务结束释放控制权，确认速度归零。
10. 再逐步增大目标距离和速度限制。

## 12. 现场建议

- 第一次上板把 `manager.max_vel`、`closed_loop_controller.max_vx` 保持在 `0.3~0.5` 更稳。
- 真机默认用 `closed_loop_controller.drive_mode=pure_pursuit`：根据当前位置在
  B-spline 上找最近点，再沿曲线前视，最后只发 `linear.x` 和 `angular.z`。
  这更接近已验证的机器狗路径跟踪 demo。需要切回旧全向输出时设置
  `CONTROLLER_DRIVE_MODE=omni`。
- 纯追踪现场先用 `CONTROLLER_PURE_PURSUIT_SPEED=0.15~0.20`、
  `CONTROLLER_LOOKAHEAD_DIST=0.45~0.60`，确认能绕障碍后再加速度。
- 保持急停/遥控器可用。
- 禁止同时运行统一 Gateway、官方 SDK demo、`zsibot_cmd_bridge` 或
  `zsibot_sdk_proxy` 中的任意两个 SDK owner；跨板状态必须由部署编排确认。
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
3. 默认只编译 `scan_planner` 需要的依赖链；旧 transport 不进入产品产物。

成功后产物在：

```bash
install-orin-sysroot/
```

可以用 `file` 确认是 ARM64：

```bash
file install-orin-sysroot/lib/scan_planner/scan_planner_node
# 仅 BUILD_LEGACY_ZSIBOT_UDP_CLIENT=1 时：
file install-orin-sysroot/lib/zsibot_cmd_bridge/zsibot_cmd_udp_client
```

正常应显示 `ELF 64-bit ... ARM aarch64`。

注意：sysroot 里的 `gcc/g++` 是 ARM64 板端原生编译器，不是 x86 可直接运行的交叉编译器；开发机上直接执行会报 `aarch64-binfmt-P: Could not open '/lib/ld-linux-aarch64.so.1'`。本仓库通过 `qemu-aarch64-static` wrapper 解决这个问题，因此编译速度会比普通交叉编译慢。
# 重要：当前真机坐标链路

本指南后文保留了早期 `lio_odom` 调试流程，便于回放旧 bag；它不再代表当前真机
默认配置。当前真机运行统一使用 Omni contract：

```text
/omni/tf_manager/ready
/omni/tf_manager/body_odom_global
/cloud_registered_global
/planning/global_path
/planning/bspline
/planning/local_path
omni_map -> omni_base_link -> omni_imu_link -> omni_lidar_link
```

请优先阅读
[`SCAN_Planner_修改清单.md`](./SCAN_Planner_修改清单.md)，并直接使用
`tools/orin_runtime/run_real_planner.sh` 或
`tools/orin_runtime/run_real_planner_udp.sh`。不要把后文旧命令中的
`lio_odom`、`lio_map`、`/state_estimation*`、`/body_state_estimation*` 原样用于
当前真机自动导航。
