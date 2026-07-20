# ZsiBot Cmd Bridge Deployment

`zsibot_cmd_bridge` provides two ways to drive a ZsiBot robot from ROS 2
velocity commands:

- Direct bridge: run the ZsiBot SDK client on the Orin NX.
- UDP proxy: run a ROS 2 UDP client on the Orin NX and a non-ROS SDK proxy on
  the RK3588. This keeps the RK3588 SDK config at `127.0.0.1`.

## Network Layout

Use these values for the current robot:

| Device | IP |
| --- | --- |
| Orin NX | `192.168.234.234` |
| RK3588 / ZsiBot motion controller | `192.168.234.1` |

## Mode A: UDP Proxy Without Editing RK3588 Config

This is the recommended first deployment mode for the current dual-board robot.
The RK3588 `/opt/export/config/sdk_config.yaml` can stay at its factory-style
local settings:

```yaml
target_ip: "127.0.0.1"
target_port: 43988
```

Runtime chain:

```text
Orin ROS 2 /scan_planner/cmd_vel
  -> zsibot_cmd_udp_client
  -> UDP 192.168.234.1:44000
  -> RK zsibot_sdk_proxy
  -> HighLevel::move(vx, vy, yaw_rate) through local SDK
```

Start the proxy on RK3588:

```bash
cd /app/rk_proxy
./run_zsibot_sdk_proxy.sh
```

Start the ROS 2 UDP client on Orin NX:

```bash
source install/setup.bash
ros2 launch zsibot_cmd_bridge zsibot_cmd_udp_client.launch.py \
  cmd_vel_topic:=/scan_planner/cmd_vel
```

Or start it together with the real planner:

```bash
ros2 launch scan_planner run.launch.py \
  is_real_world:=true \
  controller_mode:=closed_loop \
  publish_robot_description:=false \
  use_zsibot_udp_client:=true \
  real_cmd_vel_topic:=/scan_planner/cmd_vel
```

The Orin-side UDP client config is `config/zsibot_cmd_udp_client.yaml`.

## Mode B: Direct Bridge With RK3588 Config Change

The direct bridge config is `config/zsibot_cmd_bridge.yaml`:

```yaml
zsibot_cmd_bridge:
  ros__parameters:
    local_ip: "192.168.234.234"
    dog_ip: "192.168.234.1"
    local_port: 43988
```

For direct bridge mode, edit `/opt/export/config/sdk_config.yaml` on the
RK3588:

```yaml
target_ip: "192.168.234.234"
target_port: 43988
```

Restart the RK3588 motion control service or reboot the robot after changing
this file.

## Build on Orin NX

The default SDK model is `zsl-1w`.

```bash
cd ~/SCAN-Planner
source /opt/ros/humble/setup.bash

colcon build --symlink-install \
  --packages-select \
  scan_planner_msgs plan_env path_searching bspline_opt traj_utils \
  go2_description scan_planner zsibot_cmd_bridge \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

For the point-foot model, add `-DZSIBOT_MODEL=zsl-1`.

## Run

Start the planner and direct bridge together:

```bash
source install/setup.bash
ros2 launch scan_planner run.launch.py \
  is_real_world:=true \
  controller_mode:=closed_loop \
  use_zsibot_bridge:=true
```

For UDP proxy mode, use `use_zsibot_udp_client:=true` instead.

If the new robot uses different topic names, pass them at launch time:

```bash
ros2 launch scan_planner run.launch.py \
  is_real_world:=true \
  controller_mode:=closed_loop \
  use_zsibot_bridge:=true \
  use_lidar_to_body_odom:=true \
  lidar_odom_topic:=/state_estimation \
  body_odom_topic:=/body_state_estimation \
  body_odom_frame_id:=base_link \
  body_odom_sensor_frame_id:=livox_frame \
  body_odom_world_frame_id:=lio_odom \
  body_odom_publish_tf:=false \
  real_sensor_pose_topic:=/state_estimation \
  real_cloud_topic:=/cloud_registered \
  real_grid_frame_id:=lio_odom \
  real_cloud_is_world:=true \
  real_need_extrinsic:=false \
  real_depth_topic:=/your/depth/image \
  goal_frame_id:=lio_odom \
  real_cmd_vel_topic:=/scan_planner/cmd_vel
```

Topic meanings:

| Launch argument | Expected type | Used by |
| --- | --- | --- |
| `use_lidar_to_body_odom` | bool | enables `/state_estimation` lidar odom to `/body_state_estimation` conversion |
| `body_odom_topic` | `nav_msgs/Odometry` | generated body odom topic when `use_lidar_to_body_odom:=true` |
| `real_body_pose_topic` | `nav_msgs/Odometry` | planner body odom when `use_lidar_to_body_odom:=false` |
| `real_sensor_pose_topic` | `nav_msgs/Odometry` | lidar/depth sensor pose for map fusion |
| `real_cloud_topic` | `sensor_msgs/PointCloud2` | lidar obstacle input |
| `real_depth_topic` | `sensor_msgs/Image` | depth obstacle input when `sensor_type:=depth` |
| `real_cmd_vel_topic` | `geometry_msgs/Twist` | controller output and ZsiBot bridge input |
| `real_grid_frame_id` | string | occupancy map frame, usually `lio_odom` after isolating FAST_LIO from chassis odom |
| `real_cloud_is_world` | bool | `true` for FAST_LIO `/cloud_registered` |
| `real_need_extrinsic` | bool | `false` when `real_sensor_pose_topic` is already sensor pose |
| `goal_topic` | `geometry_msgs/PoseStamped` | RViz/navigation goal when `navi_mode:=1` |
| `initial_path_topic` | `nav_msgs/Path` | global path input when `navi_mode:=3` |

`body_pose` and `sensor_pose` are internal names inside SCAN-Planner. Do not
rename them in the C++ code; remap them through the launch arguments above.

## Planner Sensor Parameters

Topic names are not the only robot-specific settings. Check
`src/planner/plan_manage/config/planner.yaml` before real deployment:

| Parameter | When to change |
| --- | --- |
| `grid_map.frame_id` | Set to the world/odom frame used by the robot odometry |
| `grid_map.sensor_type` | `lidar` for point cloud, `depth` for depth image |
| `grid_map.cloud_is_world` | `true` if point cloud points are already in world frame |
| `grid_map.need_extrinsic` | `true` if `sensor_pose` is body pose and the sensor offset must be applied |
| `grid_map.cx/cy/fx/fy` | Depth camera intrinsics |
| `grid_map.k_depth_scaling_factor` | Depth image scale, usually `1000.0` for millimeters |

The current code has built-in lidar/depth extrinsic defaults in
`src/planner/plan_env/src/grid_map.cpp`. If the new robot's sensor mount is
different, update those matrices or provide a small pose conversion node that
publishes `real_sensor_pose_topic` as the actual sensor pose. The latter is
usually cleaner because SCAN-Planner can then run with `grid_map.need_extrinsic:=false`.

Or start only the direct bridge:

```bash
source install/setup.bash
ros2 launch zsibot_cmd_bridge zsibot_cmd_bridge.launch.py \
  cmd_vel_topic:=/scan_planner/cmd_vel
```

For UDP proxy mode, start only the Orin UDP client:

```bash
source install/setup.bash
ros2 launch zsibot_cmd_bridge zsibot_cmd_udp_client.launch.py \
  cmd_vel_topic:=/scan_planner/cmd_vel
```

## Smoke Test

First verify network reachability from Orin NX:

```bash
ping 192.168.234.1
ssh firefly@192.168.234.1
```

Then run either the direct bridge or the UDP client/proxy pair and publish a
small velocity command:

```bash
ros2 topic pub --rate 20 /scan_planner/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

Stop the robot:

```bash
ros2 topic pub --once /scan_planner/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

Keep the command small during first tests. Both the direct bridge and UDP proxy
republish the last command at `publish_rate` and send zero velocity when
the subscribed cmd_vel topic times out.

## Runtime Logs

The bridge prints SDK status every `status_log_period` seconds:

```text
SDK status: connected=true battery=85% ctrlmode=1 cmd=active vx=0.100 vy=0.000 yaw=0.000
```

ZSL-1w control modes from the API:

| Value | Meaning |
| --- | --- |
| `0` | damping mode |
| `1` | standing mode |
| `3` | moving mode |

If direct bridge mode prints `connected=false`, check:

- Orin IP is really `192.168.234.234`.
- RK3588 `sdk_config.yaml` has `target_ip: "192.168.234.234"`.
- No other SDK process is already using port `43988`.
- RK3588 motion control has been restarted after config changes.

If UDP proxy mode prints `connected=false` on RK3588, the Orin-to-RK UDP hop is
not the first suspect. Check that the official RK highlevel demo works with the
default local SDK config, and make sure the demo and proxy are not running at
the same time.

If `move()` returns non-zero, check that the robot is standing. The ZSL-1w API
requires `move()` to be called in standing state.

## Parameters

| Parameter | Default | Notes |
| --- | --- | --- |
| `local_ip` | `192.168.234.234` | Orin NX IP on the robot network |
| `dog_ip` | `192.168.234.1` | RK3588 / robot IP |
| `local_port` | `43988` | Must match RK3588 `target_port` |
| `publish_rate` | `50.0` | SDK command resend rate |
| `cmd_timeout` | `0.3` | Send zero if the subscribed cmd_vel topic is stale |
| `max_vx` | `0.3` | Conservative first-deployment limit |
| `max_vy` | `0.15` | Conservative first-deployment limit |
| `max_yaw_rate` | `0.5` | Conservative first-deployment limit |
| `deadband_vx` | `0.05` | Send 0 below the ZSL-1w minimum accepted x speed |
| `deadband_vy` | `0.10` | Send 0 below the ZSL-1w minimum accepted y speed |
| `deadband_yaw_rate` | `0.10` | Send 0 below the ZSL-1w minimum accepted yaw rate |
| `auto_stand` | `true` | Calls `standUp()` on startup |
| `require_standing_before_move` | `true` | Blocks `move()` until SDK reports standing/move mode |
| `standup_check_period` | `0.2` | Seconds between ctrlmode checks while standing up |
| `standup_retry_period` | `1.0` | Seconds between repeated `standUp()` requests while waiting |
| `standup_wait_timeout` | `10.0` | Seconds before warning that standUp is still not confirmed |
| `log_sdk_status` | `true` | Prints connection, battery, mode, command |
| `status_log_period` | `2.0` | Seconds between status logs |

UDP client/proxy defaults:

| Parameter / option | Default | Notes |
| --- | --- | --- |
| `proxy_ip` | `192.168.234.1` | RK3588 IP used by Orin UDP client |
| `proxy_port` / `--listen-port` | `44000` | UDP port between Orin and RK proxy |
| `--sdk-local-ip` | `127.0.0.1` | RK local SDK client IP |
| `--sdk-local-port` | `43988` | RK local SDK client port |
| `--sdk-dog-ip` | `127.0.0.1` | RK local robot SDK target |

The ZSL-1w API allows larger `move()` limits, but these defaults intentionally
keep first deployment slow. Increase the planner controller and bridge limits
together after direction, odometry, and obstacle mapping are verified.
