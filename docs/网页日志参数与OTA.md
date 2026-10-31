# 网页、日志、参数与 OTA

## 网络配置

复制 `config/local_config.example.h` 为 `local_config.h` 后填写配置。该文件被 Git 忽略。Wi-Fi 不阻塞系统初始化；不配置 Wi-Fi 时可以通过 USB 完成状态查看和运动使能。

`FOLLOW_CONTROL_TOKEN` 与 `FOLLOW_OTA_PASSWORD` 分别用于控制接口和固件升级，长度至少 16 字符。空 OTA 密码时不启动 OTA 服务。使用者自行生成独立值，不从示例中继承公共密码。

## HTTP 接口

| 接口 | 方法 | 行为 |
| --- | --- | --- |
| `/` | GET | 内嵌控制台，不依赖外部 CDN |
| `/api/status` | GET | 原始/滤波目标、数据年龄、状态、控制间隔、下发轮速、解析及发送统计 |
| `/api/control` | POST | 表单 `action=arm` 或 `action=stop` |
| `/api/params` | POST | 表单 `target_distance_cm`，停车时修改并保存 |
| `/api/log.csv` | GET | 停车时导出最近 256 个控制周期 |

写接口请求头使用 `X-Control-Token`。网页令牌仅用于当次请求，不写入浏览器本地存储。状态接口设置 `Cache-Control: no-store`，未知目标输出 JSON `null`，避免把未知距离误显示成零。

## 参数持久化

第一版网页开放目标距离一个运行参数，范围 40–1000 cm。修改必须在未使能时进行，并保持停车/恢复阈值相对目标的偏移。成功写入后保存在 NVS 命名空间 `uwb-follow-v1`；存储失败会回滚本次 RAM 修改。

重启时先加载源码中的配置，再覆盖有效的 NVS 目标距离。机械几何、最大速度和协议配置仍在配置文件管理，避免现场误改机械边界。重新刷写应用通常不会擦除 NVS；恢复源码默认目标距离，可在网页重新设置该值。

## CSV 字段

`tick_ms` 是开机相对时间；`sequence` 是日志序号；`measurement_age_ms` 是对应测量年龄，未获取测量时为 `4294967295`。原始距离/角度来自 Anchor，滤波距离/角度来自车体坐标目标。`left_command_rpm`、`right_command_rpm` 为逻辑下发值，非电机反馈。未知浮点值记为 `nan`。

默认 50 ms 周期，256 帧覆盖最近约 12.8 秒，包括停车期间。长时间停放会覆盖之前的运动数据，应在实验结束后及时导出。USB 每四个控制周期输出一条简要状态日志。

## OTA

先通过 USB 写入正确的网络配置，随后在 PowerShell 设置当前会话环境变量：

```powershell
$env:FOLLOW_OTA_HOST = "设备当前IP"
$env:FOLLOW_OTA_PASSWORD = "与本地配置一致的OTA密码"
pio run -d firmware/esp32 -e esp32-ota -t upload
```

OTA 开始回调撤销运动使能并停车，完成重启后仍然停止；失败时也不会自动恢复行驶。仓库不保存实际设备 IP、个人 Wi-Fi 或升级口令。
