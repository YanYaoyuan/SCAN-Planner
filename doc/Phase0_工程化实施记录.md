# 巡检机器狗 Phase 0 工程化实施记录

> 更新时间：2026-08-14  
> 范围：`SCAN-Planner`、`omni_slam`、`rosdeck`、`rosdeck_robot_bridge`、`omni_docking` 接口边界  
> 目标：先消除不可控运动、重复 SDK owner、保存误报和旧布局绕过，再进入任务状态机与 SLAM Manager 建设。

## 1. 当前结论

Phase 0 的代码基线已经从“多个 Demo 各自直连底盘”推进到“统一控制入口、唯一最终速度出口、故障默认停车”。当前完成的是**源码级/静态契约的软件安全候选基线**，还不能等同于“真机已验收”：官方 ZsiBot 部署模板已默认纳入独立 Safety Supervisor；关键安全 topic/service 被 remap 时会拒绝启动，心跳缺失/超时或发布者数量异常时 Bridge 会保持急停锁定。

当前最重要的边界如下：

1. `rosdeck_robot_bridge` 是产品默认唯一 ZsiBot SDK owner；
2. App、SCAN Planner、Docking 不再直接把速度送入 SDK；
3. 所有速度在 Bridge 内仲裁后，只从 `/omni/cmd_vel/final` 进入底盘 adapter；
4. 任一速度源超过 250ms 未更新、输入非有限数、发布者数量异常、控制权不匹配或 E-stop 锁定时，输出为零；
5. 旧 SCAN Bridge/SDK proxy 只允许显式兼容模式构建和运行，并有遗留产物、遗留进程与同机 SDK owner 锁三层保护；
6. SLAM 地图写盘失败会真实返回失败；重定位 launch 先做 PCD 文件级 preflight，Fast-LIO 在创建自身业务 ROS 端点前完成 PCL 解析；旧 launch 不再启动两次 ICP 或使用两份地图；
7. 官方 ZsiBot 部署模板默认启动 Gateway + Safety Supervisor，部署检查要求两者和安全心跳连续稳定；OpenNav Docking 目前只有显式启用时的 scoped `cmd_vel` remap，尚未接入 `docking-*` authority 或部署 readiness 验收。

## 2. Phase 0 接口冻结

| 功能 | 接口 | 类型 | 当前约束 |
| --- | --- | --- | --- |
| App 遥控输入 | `/omni/cmd_vel/teleop` | `geometry_msgs/msg/TwistStamped` | 需要 `app-*` 控制权；到达超时 250ms；时间戳默认用于诊断 |
| Docking 输入 | `/omni/cmd_vel/docking` | `geometry_msgs/msg/Twist` | OpenNav 的相对 `cmd_vel` 必须在 bringup 中 remap；需要 `docking-*` 控制权 |
| 导航输入 | `/scan_planner/cmd_vel` | `geometry_msgs/msg/Twist` | 需要 `mission-*` 控制权；到达超时 250ms |
| 软件 E-stop 心跳 | `/omni/safety/estop` | `std_msgs/msg/Bool` | 必须恰好一个发布者；`true` 立即锁定；`false` 只表示健康，不自动解锁 |
| E-stop 请求 | `/omni/safety/estop_request` | `std_msgs/msg/Bool` | 由上层或硬件适配层请求安全监控重新锁定 |
| Supervisor 解锁 | `/omni/safety/arm_supervisor` | `std_srvs/srv/Trigger` | 独立第一阶段；成功后只恢复健康 `false` 心跳，不自动清除 Bridge latch |
| Supervisor 重新锁定 | `/omni/safety/latch_estop` | `std_srvs/srv/Trigger` | 主动重新锁定 Supervisor；不等同于硬件急停 |
| E-stop 复位 | `/omni/safety/reset_estop` | `std_srvs/srv/Trigger` | 仅在安全心跳唯一、最新值为 `false` 且新鲜，并且需租约的 adapter 已确认直接 stop 时允许复位 Bridge latch |
| Supervisor 状态 | `/omni/safety/supervisor_status` | `std_msgs/msg/String` | 周期心跳，包含 armed/latched、输出 E-stop 和健康心跳时效；Phase 1 改为强类型 |
| 最终速度 | `/omni/cmd_vel/final` | `geometry_msgs/msg/Twist` | 仅 Bridge 内部 arbiter 发布；ZsiBot adapter 要求发布者数量恰好为 1 |
| 仲裁状态 | `/omni/cmd_vel/arbiter_status` | `std_msgs/msg/String` | 临时诊断接口；Phase 1 迁移为强类型消息 |
| 电池状态 | `/battery_state` | `sensor_msgs/msg/BatteryState` | 1s 发布缓存；Zsi SDK SOC 默认 10s 采样；percentage 未知/过期以及未接入的电压/电流/温度等为 `NaN` |
| 标准诊断 | `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | 底盘连接、遥测时效、模式、姿态、owner 和 SDK 错误 |
| Adapter 摘要 | `/omni/robot/adapter_status` | `std_msgs/msg/String` | Phase 0 临时人机诊断接口；Phase 1 并入强类型 `RobotState` |
| Adapter 细分状态 | `/omni/robot/connection`、`/omni/robot/mode`、`/omni/robot/sdk_error` | `std_msgs/msg/String` | 临时的 known/fresh/age/error 诊断；不用作安全授权 |
| 控制命令/状态 | `/rosdeck/control_command`、`/rosdeck/control_status` | `std_msgs/msg/String` | 当前为租约协议；Phase 1 增加 epoch/token 并改为强类型 |
| 姿态/步态命令 | `/rosdeck/posture_command`、`/rosdeck/locomotion_command` | `std_msgs/msg/String` | 当前命令携带 `client_id`，Bridge 校验必须等于当前 owner |

说明：Phase 0 使用“固定角色优先级 + 控制权互斥”。`app-*` 高于 `docking-*`，`docking-*` 高于 `mission-*`；高优先级接管时先显式发送零速，再切换 owner。它不是防伪授权协议，ROS 图中的恶意发布者仍可能伪造公开 client ID，必须在 Phase 1 用不可猜 lease token、epoch 和访问控制收口。

## 3. 已实现内容

### 3.1 唯一 SDK owner 与旧 Bridge 下线

- 产品默认不再由 SCAN launch 启动 `zsibot_cmd_bridge`；
- 旧 direct bridge、UDP client、SDK proxy 都增加显式兼容模式开关；
- “打包 UDP 兼容客户端”和“编译会持有厂商 SDK 的旧 target”拆成两个独立构建开关；
- 标准产品产物会检查并拒绝旧 `zsibot_cmd_bridge`、`zsibot_sdk_proxy` 遗留二进制；
- 产品启动会区分 `pgrep` 的无匹配和枚举失败，无法证明安全时拒绝启动；
- 交叉编译使用 `--artifacts-only`，不会依赖构建主机进程枚举，也不会误扫宿主 overlay；
- 新旧 SDK owner 使用同一个非阻塞 `flock`；默认锁放在 `/run/lock/omni/`，拒绝 symlink、非普通文件、非当前 owner、多硬链接以及组/其他用户可写的锁文件；
- 锁覆盖同一台主机的进程。RK 与 Orin 分板部署不会共享这把锁，仍必须由部署拓扑保证只启用一个跨板 SDK owner。

### 3.2 统一速度仲裁与底盘安全

- Bridge 内接入 teleop、docking、navigation 三种速度源与独立 E-stop 监控；
- ZsiBot 产品配置禁止关闭 arbiter；
- 输入 topic 在 ROS namespace/remap 解析后再次校验，防止相对名或 remap 形成 `/omni/cmd_vel/final` 自反馈；
- 每个速度输入要求恰好一个 ROS publisher，冲突时清除该源并输出零；
- 最终 adapter 也要求 `/omni/cmd_vel/final` 恰好一个发布者；
- 对 `NaN/Inf`、轴限速、最小 deadband、零速、SDK 返回错误和 SDK 异常建立统一处理；
- 非零 SDK move 失败后，在额外日志或诊断调用前立即发送显式零速；零速失败时保留“可能仍在运动”状态并由 watchdog 重试；
- App/导航/Docking 到达超时默认 250ms，参数被限制在 100~300ms；
- SDK 释放、断连、租约超时、Bridge 析构和高优先级抢占均先走安全停车；释放过程中卧下/被动姿态未确认时返回 degraded failure，不再误报安全释放成功；
- SDK 调用异常进入 fault/cooldown 并释放 owner，避免普通异常直接杀死 Node。
- 软件 E-stop 新锁定时除 arbiter 零速输出外，还绕过普通命令通道直接请求 adapter stop；失败以 200ms 间隔有界重试且去洪泛，未确认停车时禁止普通 E-stop reset，adapter 自身 watchdog 继续承担末端停车重试。

### 3.3 控制权和 App 兼容迁移

- App 新默认发布 `/omni/cmd_vel/teleop` 的 `TwistStamped`；
- 既有 ZsiBot `/vel_cmd` 布局只有在运行时检测到统一 Gateway 能力后才迁移，旧 VBot 不被静态强迁；
- Foxglove 下用 Gateway 主动发布的 `/omni/cmd_vel/arbiter_status` 作为能力证明，避免订阅型 teleop topic 在 graph 中不可见；
- 连接初始化、topic 探测、机器人快速切换和建议弹窗均绑定 URL 与 transport generation，避免旧连接的异步结果污染新机器人；
- 发布循环每次实时读取控制权；非 owner 连零速也不发布，避免第二台手机覆盖真正 owner；
- roslib publisher 同时校验 topic、message type 和 ROS 实例，关闭 render/effect 切换窗口；
- Demo 连接单独允许无硬件演示，真实但不支持 authority 的统一设备保持 fail-closed；
- 姿态和 locomotion 命令携带当前 App client ID，Bridge 校验它与实际 lease owner 一致。
- App 安全面板同时订阅 Supervisor 和 Arbiter 状态，只有当前 App 确实持有控制权、连接与两路遥测都有效时，才允许进入两次独立确认的 `arm -> reset` 流程；任一步失败、取消或身份变化都不会自动继续。

### 3.4 SLAM 回归修复

- PCD 保存函数返回真实成功/失败；目录创建、空路径、写盘失败和异常均会反馈到 `/map_save`；
- `/map_save` 不再无条件返回 success；
- `relocalization.launch.py` 删除直接启动的重复 ICP，只保留延迟启动组；
- 定位模式强制 prior map 为绝对路径：launch 先检查 `.pcd` 后缀、存在、可读与非空；Fast-LIO 构造期再在创建自身业务 publisher/subscriber/timer/service 前完成 PCL 解析、空点云和 voxel 后空点云检查，失败 `FATAL + throw`；
- 新旧两套重定位 launch 都使用一个必填 `map_path`，同时传给 ICP 和 Fast-LIO，防止两套坐标系读取不同地图；
- 增加源码级/AST 回归，防止保存结果被丢弃、ICP 重复加入、默认空地图启动或关键定位进程异常后 launch 假健康；退出契约尚需 Humble `launch_testing` 真实运行验证。

### 3.5 Safety Supervisor、产品启动与部署健康

- Supervisor 启动时默认锁定并周期发布 `true`；只有显式 `arm` 后才持续发布健康 `false`，任一 E-stop request 会重新锁定；
- ZsiBot 配置不允许关闭 arbiter 或 E-stop monitor，monitor deadline 不允许超出 100~500ms 产品边界；
- Bridge 强制解析后的 E-stop topic/reset service，Supervisor 强制 E-stop output/request topic，避免关键安全链被参数覆盖或 ROS remap 拆开；arm/latch/status 名称仍由官方产品配置固定；
- `product_bringup.launch.py` 作为统一入口；官方 ZsiBot 部署模板默认同时启动 Gateway 与 Supervisor。OpenNav Docking 默认关闭，显式启用时只把其相对 `cmd_vel` 在该 scope 内 remap 到 docking 输入；
- 关键进程退出会结束整个 launch epoch，交由 systemd 整体重启，不再由内部 `respawn` 反复短暂注册 ROS 节点；
- 部署验收不再只看一次 `ros2 node list`：它会检查 systemd MainPID/cgroup、Gateway + Supervisor 进程归属与同时稳定、Supervisor heartbeat 序列持续前进以及 Arbiter 无 monitor fault。它目前不验证可选 Docking 节点或 lifecycle readiness。

### 3.6 底盘标准遥测与状态时效

- Robot Adapter 新增只读快照契约；Bridge 的状态发布定时器只读缓存，不调用厂商 SDK，避免诊断 I/O 阻塞 watchdog/E-stop 回调；
- 每 1s 发布 `/battery_state` (`sensor_msgs/msg/BatteryState`) 与 `/diagnostics` (`diagnostic_msgs/msg/DiagnosticArray`)。当前 Zsi SDK 仅提供默认 10s 采样的 SOC 百分比；该 percentage 未知/过期时为 `NaN`，电压/电流/温度/容量等未接入字段也保持 `NaN`，status/health/technology 保持 `UNKNOWN`；
- 同时发布 `/omni/robot/connection`、`/omni/robot/mode`、`/omni/robot/sdk_error` 和 `/omni/robot/adapter_status`，显式携带 known/fresh/age/error/owner/sequence；
- ZsiBot 的连接、控制模式、姿态、电量和 SDK 错误由原有串行 SDK 路径刷新缓存；不支持的测量维度保持 `unknown`。VBot/ZsiBot 对“配置上存在电池”作 adapter 级声明，这不是充电器连接或权威 BMS presence 信号；
- telemetry、battery、motion input、motion SDK、stop、release、locomotion、posture 和 authority 故障分域独立锁存；普通连接/模式轮询恢复只清自己的 telemetry 域，不会把尚未解决的运动、停车或释放故障误抹成健康；
- Arbiter status 已改为最长 1s 一次的单调 `status_seq` 心跳；App 对 Supervisor 和 Arbiter 任一路超过 3s 未刷新、序列非法或 monitor fault 均 fail-closed，禁止复位。

## 4. 当前安全状态机

```text
Safety Supervisor 未启动/未 arm/请求急停
                |
                v
    /omni/safety/estop = true 或心跳缺失
                |
                v
       Bridge E-stop latch = true
                |
                +--> arbiter 输出零
                +--> adapter 显式 stop
                +--> 拒绝新姿态/步态请求

Safety Supervisor 显式 arm 且持续健康心跳(false)
                |
                v
操作员/上层调用 /omni/safety/reset_estop
                |
                v
Bridge 在“唯一 + 新鲜 + false + 需租约 adapter 的 direct stop 已确认”成立时解除 latch
```

`false` 心跳不能自动清除 latch。这是为了避免安全监控短暂恢复后机器人自行重新开始运动。复位后各速度源的旧命令已经清空，必须重新获得控制权并发送新命令。

## 5. 已执行验证

本地不具备 ROS 2 Humble、colcon、完整 gtest 和 ZsiBot 真机，因此当前验证分为纯逻辑、静态接线和 App 全量测试：

| 范围 | 结果 |
| --- | --- |
| SCAN 旧 Bridge 隔离回归 | 7/7 通过 |
| SLAM Phase 0 回归 | 6/6 通过 |
| App Jest 全量回归 | 33 suites、260 tests 通过 |
| App TypeScript | `tsc --noEmit` 通过 |
| Bridge 产品 bringup / Supervisor / 遥测静态契约 | 34/34 通过 |
| 速度安全与仲裁头文件 | C++17 严格语法检查通过 |
| BridgeNode 接线 | 使用最小 ROS 接口桩语法检查通过 |
| ZsiBot adapter | 使用实际 SDK 头和 ROS 接口桩语法检查通过 |
| Launch/Shell/XML | Python 语法、Shell 语法、package XML 解析通过 |
| 补丁质量 | 三个仓库 `git diff --check` 通过 |

纯逻辑用例覆盖：三个速度源、固定优先级选择、owner gate、250ms 超时、时间戳 stale/future/非有限数、E-stop latch/reset 清旧命令、直接 stop 事件隔离与有界重试、SDK move success/error/exception、watchdog、Adapter Snapshot 时效、Battery percentage/presence 语义和独立故障域。SDK owner 锁测试已升级为父子进程竞争，验证第一个进程释放前第二个进程无法取得锁。

## 6. 尚未完成的 Phase 0 项

### P0：上真机前必须完成

1. **Docking authority 真实接线**：OpenNav 的速度 remap 已进入产品 bringup，但 Docking Manager 还必须以 `docking-*` 身份申请、维持和释放控制权，并覆盖失败/抢占路径；
2. **Orin/Humble 完整构建**：用实际 ROS 消息、ZsiBot SDK 和目标 aarch64 环境完成 `colcon build`、launch 集成、DDS graph 契约和依赖检查；
3. **整机参数单一事实源**：在产品 bringup 固定 Domain ID、RMW、frame、外参、速度限制、地图与路线版本，禁止各个脚本继续各自覆盖；
4. **真机故障注入**：逐项测试 App 断网、Planner kill、Bridge kill/restart、Safety Supervisor kill、SDK 断开、底盘拒绝 move、E-stop 发布者掉线和多发布者冲突，并测量真实停车延迟；
5. **硬件急停方案**：软件 Bool topic 不能替代硬件安全回路；已经执行中的 `standUp/lieDown` 也未必能被速度零安全中断；
6. **SDK 阻塞测量与兜底**：异常已捕获，但阻塞调用仍可卡住同一进程的 watchdog。必须实测最坏阻塞时间，并确认下位机自带命令超时；必要时将 SDK I/O 隔离到独立进程。

### P1：Phase 1 紧接着完成

1. 建立 `omni_robot_interfaces`，替换当前 String/Bool 临时协议；
2. 控制租约加入不可猜 token、epoch、sequence 和过期时间，所有速度/姿态/步态命令精确绑定租约；
3. 在 Phase 0 底盘标准遥测基础上实现强类型 `RobotState`，继续聚合最终 owner、定位质量、充电状态和安全状态；
4. 实现 RobotState 与 Mission Manager 状态机；
5. 将 SCAN 路线执行包装为可取消、有 feedback/result 的 Action；
6. 建立地图、路线、标定版本绑定和持久化任务事件。

## 7. 真机验收步骤

按以下顺序执行，任何一步失败都不得继续放开运动：

1. 在干净 install prefix 构建，确认旧 SDK owner 产物检查通过；
2. 确认同机没有旧 bridge/proxy 进程，RK/Orin 跨板部署只保留一个 SDK owner；
3. 启动 Safety Supervisor，未 arm 时 `/omni/safety/estop` 必须为 `true`；
4. 启动 Bridge，确认 arbiter 状态为 E-stop latched，底盘收到零速；
5. 显式 arm Safety Supervisor，确认持续、唯一、周期 `false` 心跳；
6. 调用 `/omni/safety/reset_estop`，确认旧速度源不会恢复；
7. App 申请控制权并进入 locomotion，低速前后左右与旋转逐轴测试；
8. 运动中依次断开 App 网络、停止 teleop 发布、kill App、kill Bridge，记录从最后非零命令到实测停车的时间；
9. 运行 SCAN 路线，验证 mission owner；随后 App 接管，确认先停车再抢占；
10. 运行 Docking，验证 remap、docking owner、App 抢占和 E-stop；
11. 注入两个 teleop publisher、两个 E-stop publisher和 E-stop 心跳丢失，均必须锁停；
12. 重启所有进程，确认没有自动恢复旧速度、旧姿态动作或旧控制权。

## 8. 下一步实施顺序

1. 完成 Docking authority client 和抢占/释放集成测试；
2. 在 Orin/Humble 做完整构建与 graph contract 测试；
3. 进行桌面架空测试，再进行低速落地故障注入；
4. 接入硬件急停、四足安全姿态和充电/底盘真实健康信号；
5. 进入 Phase 1：`omni_robot_interfaces`、RobotState、Mission Manager、SLAM Manager。

在完成第 4 步之前，当前版本的正确定位是“Phase 0 软件安全候选基线”，不是可无人值守部署版本。
