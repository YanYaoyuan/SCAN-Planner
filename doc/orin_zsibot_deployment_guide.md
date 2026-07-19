# SCAN-Planner + ZsiBot Orin NX 部署手册

本文档面向当前双板机器狗：

- Orin NX：运行 SCAN-Planner、LIO/感知、`zsibot_cmd_bridge`
- RK3588：运行 ZsiBot 原厂运动控制程序
- Orin NX IP：`192.168.234.234`
- RK3588 IP：`192.168.234.1`
- SDK 端口：`43988`
- 默认型号：`zsl-1w`

## 1. 总体链路

```text
LIO / Sensor Driver
  -> odom / sensor_pose / cloud / depth

SCAN-Planner on Orin NX
  -> /cmd_vel

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

## 4. RK3588 SDK 通信配置

从 Orin NX 登录 RK3588：

```bash
ssh firefly@192.168.234.1
```

修改 RK3588 上的 SDK 配置：

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
  --packages-up-to scan_planner zsibot_cmd_bridge \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

说明：

- `--packages-up-to scan_planner zsibot_cmd_bridge` 会编译这两个包及其工作区依赖。
- 默认编译 `zsl-1w`。
- 如果以后换成点足版本，增加 `-DZSIBOT_MODEL=zsl-1`。

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

如果新狗话题名正好和默认一致：

```bash
source /opt/ros/humble/setup.bash
source ~/SCAN-Planner/install/setup.bash

ros2 launch scan_planner run.launch.py \
  is_real_world:=true \
  sensor_type:=lidar \
  controller_mode:=closed_loop \
  use_zsibot_bridge:=true
```

如果话题名不同，启动时传参：

```bash
ros2 launch scan_planner run.launch.py \
  is_real_world:=true \
  sensor_type:=lidar \
  controller_mode:=closed_loop \
  use_zsibot_bridge:=true \
  real_body_pose_topic:=/your/robot/odom \
  real_sensor_pose_topic:=/your/lidar/odom \
  real_cloud_topic:=/your/lidar/points \
  real_cmd_vel_topic:=/cmd_vel
```

深度相机模式：

```bash
ros2 launch scan_planner run.launch.py \
  is_real_world:=true \
  sensor_type:=depth \
  controller_mode:=closed_loop \
  use_zsibot_bridge:=true \
  real_body_pose_topic:=/your/robot/odom \
  real_sensor_pose_topic:=/your/camera/odom \
  real_depth_topic:=/your/depth/image \
  real_cmd_vel_topic:=/cmd_vel
```

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

## 9. 单独测试 ZsiBot Bridge

不启动 planner，只测试 SDK 控制链路：

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

Bridge 默认 50Hz 重发最近速度，`cmd_timeout` 后会发零速。

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

### 10.3 Bridge 日志 connected=false

重点检查：

```bash
ping 192.168.234.1
ssh firefly@192.168.234.1
```

RK3588 上检查：

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
- 看 bridge 日志里的 `ctrlmode`
- 停掉其他 SDK demo
- 重启 RK3588 运动控制程序

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
2. RK3588 配好 `sdk_config.yaml` 并重启运动控制。
3. Orin 编译通过。
4. 单独启动 `zsibot_cmd_bridge`。
5. 手动发小 `/cmd_vel`，确认站立、前后、左右、旋转方向。
6. 启动 LIO/点云，确认 odom 和 cloud 频率。
7. 启动 planner，但先不要给目标点，观察是否有 odom/map。
8. 给很近的目标点，低速测试。
9. 再逐步增大目标距离和速度限制。

## 12. 现场建议

- 第一次上板把 `manager.max_vel`、`closed_loop_controller.max_vx` 保持在 `0.3~0.5` 更稳。
- 保持急停/遥控器可用。
- 避免同时运行官方 SDK demo 和 `zsibot_cmd_bridge`。
- 每次改 RK3588 SDK 配置后都重启运动控制。
- 如果现场网络不稳定，先不要跑 planner，先用 bridge 冒烟测试。
