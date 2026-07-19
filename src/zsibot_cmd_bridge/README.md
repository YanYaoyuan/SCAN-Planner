# ZsiBot Cmd Bridge Deployment

`zsibot_cmd_bridge` runs on the Orin NX. It subscribes to ROS 2 `/cmd_vel`
and sends `HighLevel::move(vx, vy, yaw_rate)` commands to the ZsiBot motion
controller on the RK3588.

## Network Layout

Use these values for the current robot:

| Device | IP |
| --- | --- |
| Orin NX | `192.168.234.234` |
| RK3588 / ZsiBot motion controller | `192.168.234.1` |

The bridge config is `config/zsibot_cmd_bridge.yaml`:

```yaml
zsibot_cmd_bridge:
  ros__parameters:
    local_ip: "192.168.234.234"
    dog_ip: "192.168.234.1"
    local_port: 43988
```

On the RK3588, edit `/opt/export/config/sdk_config.yaml`:

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
  --packages-up-to scan_planner zsibot_cmd_bridge \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

For the point-foot model, add `-DZSIBOT_MODEL=zsl-1`.

## Run

Start the planner and bridge together:

```bash
source install/setup.bash
ros2 launch scan_planner run.launch.py \
  is_real_world:=true \
  controller_mode:=closed_loop \
  use_zsibot_bridge:=true
```

If the new robot uses different topic names, pass them at launch time:

```bash
ros2 launch scan_planner run.launch.py \
  is_real_world:=true \
  controller_mode:=closed_loop \
  use_zsibot_bridge:=true \
  real_body_pose_topic:=/your/robot/odom \
  real_sensor_pose_topic:=/your/lidar/odom \
  real_cloud_topic:=/your/lidar/points \
  real_depth_topic:=/your/depth/image \
  real_cmd_vel_topic:=/cmd_vel
```

Topic meanings:

| Launch argument | Expected type | Used by |
| --- | --- | --- |
| `real_body_pose_topic` | `nav_msgs/Odometry` | planner FSM, sliding map, closed-loop controller |
| `real_sensor_pose_topic` | `nav_msgs/Odometry` | lidar/depth sensor pose for map fusion |
| `real_cloud_topic` | `sensor_msgs/PointCloud2` | lidar obstacle input |
| `real_depth_topic` | `sensor_msgs/Image` | depth obstacle input when `sensor_type:=depth` |
| `real_cmd_vel_topic` | `geometry_msgs/Twist` | controller output and ZsiBot bridge input |
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

Or start only the bridge:

```bash
source install/setup.bash
ros2 launch zsibot_cmd_bridge zsibot_cmd_bridge.launch.py
```

## Smoke Test

First verify network reachability from Orin NX:

```bash
ping 192.168.234.1
ssh firefly@192.168.234.1
```

Then run the bridge and publish a small velocity command:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

Stop the robot:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

Keep the command small during first tests. The bridge republishes the last
command at `publish_rate` and sends zero velocity when `/cmd_vel` times out.

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

If `connected=false`, check:

- Orin IP is really `192.168.234.234`.
- RK3588 `sdk_config.yaml` has `target_ip: "192.168.234.234"`.
- No other SDK process is already using port `43988`.
- RK3588 motion control has been restarted after config changes.

If `move()` returns non-zero, check that the robot is standing. The ZSL-1w API
requires `move()` to be called in standing state.

## Parameters

| Parameter | Default | Notes |
| --- | --- | --- |
| `local_ip` | `192.168.234.234` | Orin NX IP on the robot network |
| `dog_ip` | `192.168.234.1` | RK3588 / robot IP |
| `local_port` | `43988` | Must match RK3588 `target_port` |
| `publish_rate` | `50.0` | SDK command resend rate |
| `cmd_timeout` | `0.3` | Send zero if `/cmd_vel` is stale |
| `max_vx` | `0.75` | Conservative planner-side limit |
| `max_vy` | `0.35` | Conservative planner-side limit |
| `max_yaw_rate` | `1.0` | Conservative planner-side limit |
| `auto_stand` | `true` | Calls `standUp()` on startup |
| `log_sdk_status` | `true` | Prints connection, battery, mode, command |
| `status_log_period` | `2.0` | Seconds between status logs |

The ZSL-1w API allows larger `move()` limits, but these defaults match the
planner controller limits and are safer for first deployment.
