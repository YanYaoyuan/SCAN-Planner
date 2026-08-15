# dog3 离线依赖说明

这些文件从 Desktop dog3 部署快照原样并入，仅由 `2_build_dog3.sh`
显式启用；普通 Omni/CI 构建通过 `src/dog3_vendor/COLCON_IGNORE` 使用系统
ROS 2 Humble 依赖。

| 组件 | 快照版本 | 上游仓库 | 许可证 |
| --- | --- | --- | --- |
| `pcl_msgs` | 1.0.0 | https://github.com/ros-perception/pcl_msgs | BSD-3-Clause |
| `pcl_conversions` | 2.4.5 | https://github.com/ros-perception/perception_pcl | BSD |
| `cv_bridge` | 3.2.1 | https://github.com/ros-perception/vision_opencv | Apache-2.0 / BSD |
| `colcon` wheels | 见各 wheel 文件名 | https://github.com/colcon | 各包自带元数据 |

原 Desktop 快照没有保存三个 ROS 包的上游 commit。升级这些依赖时必须整体
替换并在 dog3 ARM64 镜像中重新执行 `python3 -m pip check`、九包构建和
运行时 `ldd` 检查，不能只替换单个头文件或共享库。
