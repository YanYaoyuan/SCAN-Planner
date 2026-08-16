# ZsiBot RK3588 SDK Proxy

> **已弃用，仅兼容迁移使用。** 产品运行时应由统一 robot bridge 独占厂商
> SDK。下面的旧 proxy 需要显式设置确认变量，且不得与统一 bridge 同时运行。

这个目录用于“不修改 RK3588 `/opt/export/config/sdk_config.yaml`”的双板控制方案。

运行位置：

- Orin NX：运行 SCAN-Planner 和 `zsibot_cmd_udp_client`
- RK3588：运行本目录的 `zsibot_sdk_proxy`

通信链路：

```text
Orin /cmd_vel
  -> UDP 192.168.234.1:44000
  -> RK zsibot_sdk_proxy
  -> ZsiBot SDK 127.0.0.1:43988 -> 127.0.0.1
```

RK3588 的 `/opt/export/config/sdk_config.yaml` 可以保持默认：

```yaml
target_ip: "127.0.0.1"
target_port: 43988
```

## 启动

把本目录拷到 RK3588 后运行：

```bash
cd rk_proxy
ENABLE_DEPRECATED_ZSIBOT_TRANSPORT=1 ./run_zsibot_sdk_proxy.sh
```

正常日志应包含：

```text
ZsiBot SDK proxy listening on 0.0.0.0:44000
Connecting SDK locally: 127.0.0.1:43988 -> 127.0.0.1
standUp() returned 0
Waiting for standUp state before sending move commands
standUp state confirmed: ctrlmode=1; move commands enabled
```

如果一直是：

```text
SDK status: connected=false battery=0% ctrlmode=0
```

说明 RK 本机 SDK 与运动控制程序仍然没通。此时先在 RK 上确认官方 highlevel demo 是否能用默认 `127.0.0.1` 控制。
