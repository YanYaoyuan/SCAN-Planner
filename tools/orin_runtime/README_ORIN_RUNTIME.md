# SCAN-Planner Orin NX Runtime Package

这个包是给 Orin NX 运行用的 ARM64 ROS 2 install 产物。

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
ros2 pkg list | grep zsibot_cmd_bridge
```

确认产物是 ARM64：

```bash
file install/lib/scan_planner/scan_planner_node
file install/lib/zsibot_cmd_bridge/zsibot_cmd_bridge
```

## 3. 先测试 Orin 到 RK3588 网络

按当前机器狗网络：

```bash
ping 192.168.234.1
ssh 192.168.234.1
```

Orin NX 的控制网 IP 默认按 `192.168.234.234` 配置，RK3588 默认按 `192.168.234.1` 配置。

## 4. 只启动控制桥接

先不要启动 planner，单独测试 `/cmd_vel` 到 SDK：

```bash
./run_bridge_only.sh
```

另开终端发小速度：

```bash
source /opt/ros/humble/setup.bash
source /app/scan_planner_orin_nx_aarch64_20260719/install/setup.bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.05, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

确认机器狗方向无误后，再测试 `y` 和 `yaw`。当前桥接默认速度上限是 `vx=0.3 m/s`、`vy=0.15 m/s`、`yaw=0.5 rad/s`。

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

当前启动脚本使用的主要映射：

- `publish_robot_description:=false`
- `real_body_pose_topic:=/state_estimation`
- `real_sensor_pose_topic:=/state_estimation`
- `real_cloud_topic:=/cloud_registered`
- `real_grid_frame_id:=odom`
- `real_cloud_is_world:=true`
- `real_need_extrinsic:=false`
- `use_zsibot_bridge:=true`

如果新狗上 LIO、TF、点云话题不同，优先改 `run_real_planner.sh` 里的这些 launch 参数。

## 6. 录制并运行 waypoint 路线

确认 `/state_estimation` 正常后，启动记录器：

```bash
cd /app/scan_planner_orin_nx_aarch64_20260719
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run scan_planner keypoint_recorder.py \
  --odom /state_estimation \
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

这些 waypoint 是 `odom` 坐标系下的绝对坐标。

使用录好的点运行 `navi_mode=2`：

```bash
ros2 launch scan_planner run.launch.py \
  is_real_world:=true \
  navi_mode:=2 \
  keypoints_file:=/app/scan_planner_orin_nx_aarch64_20260719/keypoints.yaml \
  sensor_type:=lidar \
  controller_mode:=closed_loop \
  publish_robot_description:=false \
  use_zsibot_bridge:=true \
  real_body_pose_topic:=/state_estimation \
  real_sensor_pose_topic:=/state_estimation \
  real_cloud_topic:=/cloud_registered \
  real_grid_frame_id:=odom \
  real_cloud_is_world:=true \
  real_need_extrinsic:=false
```
