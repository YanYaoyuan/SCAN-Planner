# SCAN-Planner offline bag validation

这些脚本用于在本机或 Orin 上复现“只回放算法输入，由当前 SCAN-Planner 重新生成输出”的流程。

默认输入和地图：

```bash
BAG_PATH=/home/user/robot/data/gangbeng/rosbag2_1970_01_01-09_53_08
PCD_MAP_FILE=/home/user/robot/SCAN-Planner/scans.pcd
ROS_DOMAIN_ID=73
GRID_FRAME_ID=lio_odom
```

## 一键跑一轮

```bash
cd /home/user/robot/SCAN-Planner
tools/offline_validation/run_all_once.sh
```

默认会：

- 启动 `scan_planner run.launch.py`，不启动 `zsibot_cmd_bridge`
- 从 bag 只回放 `/state_estimation` 和 `/cloud_registered`
- 等 `/body_state_estimation` 到达
- 发布一个测试 goal
- 录制 Planner 重新生成的输出到 `/tmp/scanplanner_offline_outputs_时间戳`

常用覆盖：

```bash
BAG_PATH=/path/to/bag \
PCD_MAP_FILE=/path/to/scans.pcd \
GOAL_X=2.38 GOAL_Y=4.59 GOAL_Z=0.2 \
RUN_SECONDS_AFTER_GOAL=45 \
tools/offline_validation/run_all_once.sh
```

## 分终端调试

终端 1：启动 Planner。

```bash
tools/offline_validation/start_planner_only.sh
```

终端 2：只播放输入 bag。

```bash
tools/offline_validation/play_input_bag_only.sh
```

暂停/恢复输入播放：

```bash
tools/offline_validation/pause_input_replay.sh
tools/offline_validation/resume_input_replay.sh
```

也可以让输入播放一开始就暂停，等 Planner、RViz、录包都准备好以后再恢复：

```bash
tools/offline_validation/play_input_bag_only.sh --start-paused
```

终端 3：录制当前 Planner 输出。

```bash
tools/offline_validation/record_outputs.sh
```

终端 4：打开 RViz。

```bash
tools/offline_validation/start_rviz_only.sh
```

如果还想在 RViz 里显示 `scans.pcd` 全局地图参照，另开一个终端：

```bash
tools/offline_validation/publish_pcd_map.sh
```

如果 RViz Fixed Frame 还是默认 `world`，而输出都是 `lio_odom`，可以临时补一个可视化 TF：

```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 world lio_odom
```

终端 5：发 goal。

```bash
tools/offline_validation/publish_goal.sh
```

## 播放验证输出

```bash
tools/offline_validation/play_output_bag_only.sh /tmp/scanplanner_offline_outputs_时间戳
```

这个脚本只播放输出 bag，不会启动 RViz。

如果想一条命令同时启动 RViz、发布 `scans.pcd`、补 `world -> lio_odom` 静态 TF、并循环播放输出 bag，可以用集成版：

```bash
tools/offline_validation/play_outputs_with_rviz.sh /tmp/scanplanner_offline_outputs_时间戳
```

集成版会同时启动：

- 临时 `world -> lio_odom` 单位静态 TF，方便默认 RViz 配置显示
- `/map_generator/global_cloud`，来自 `scans.pcd`
- RViz
- 输出 bag 循环播放

如果你的 RViz Fixed Frame 已经改成 `lio_odom`，或你的系统里已经有 `world` 相关 TF，可以关闭临时 TF：

```bash
PUBLISH_WORLD_ALIAS=false tools/offline_validation/play_outputs_with_rviz.sh /tmp/xxx
```

## 绘制轨迹点图

绘制 bag 里 `/body_state_estimation.pose.pose.position.x/y` 形成的二维坐标点图：

```bash
tools/offline_validation/plot_body_xy.sh
```

默认输出到 `/tmp/rosbag2_1970_01_01-09_53_08_body_state_estimation_xy.png`。

指定 bag、图片和 CSV：

```bash
tools/offline_validation/plot_body_xy.sh /path/to/bag \
  -o /tmp/body_xy.png \
  --csv /tmp/body_xy.csv
```

## 注意

- 离线输入回放默认使用 `tools/offline_validation/replay_input_only.py`，不是 `ros2 bag play --topics`。这是因为这个 bag 上实际遇到过 `rosbag2_player` 节点存在、但输入 topic publisher 数为 0 的情况。
- `scans.pcd` 在这套真机输入回放流程中主要用于 RViz 全局地图参照；Planner 的局部栅格仍来自回放的 `/cloud_registered`。
- 如果 `REPLAY_LOOP=true`，bag 播放到末尾会跳回开头，位姿也会跳回开头；这会触发 Planner 重新规划，属于离线循环回放副作用。
