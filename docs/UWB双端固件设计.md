# CBU5000V210 UWB 双端固件设计

## 工程组织与角色

`tag_firmware` 是 initiator，安装在随身端；`anchor_firmware` 是 responder，安装在车端。两端共享 `shared_sdk`，各自保留 App、Keil 工程和链接配置。当前版本继承已有双端固件的业务实现。

硬件方案为 **Tag 全向单天线，Anchor 三天线接收阵列**。Tag 完成收发和测距交互，Anchor 用多通道接收相位测方向。选型依据、PDoA/AoA 关系及误差边界见[天线选型与测距测角原理](天线选型与测距测角原理.md)。

共享 framework 的 `UWB_ANTENNA_TYPE_CONFIG` 在 Tag 工程仍选择 `DEF_ANTENNA_TYPE_TRIANGLE_UP`，Anchor 选择 `DEF_ANTENNA_TYPE_TRIANGLE_DOWN`。Tag 保留的 SDK 阵列配置名称不代表其本端运行了三天线测角；其应用收包走 RX0。Anchor 的实际阵列端口和安装方向需要配套核对，不能将布局宏作为随意交换左右方向的开关。

## 测距测角职责与计算顺序

本工程中，距离采用 DS-TWR，Tag 在获取 Anchor 时序后计算并回传；角度由 Anchor 采集三路 CIR、计算三对 PDoA 统计值后解算。当前用于控制的角度仅取 RX1–RX2 的 `PD12`，不是三对 PDoA 融合的完整三维 AoA。

实际步骤为：5 包统计 → PD12 减去参考偏置 77.09° → 相位折返 → 使用波长 3.753 cm、基线 1.88 cm 计算 `asin` → 大跳变保持 → 输出 A。`s_eleResult=0` 表示本版不输出实测俯仰角。

SDK 存在完整偏置补偿/三维 LUT 接口，但本版 Anchor App 使用上述简化路径。无线交互、超时和串口后的每层处理见[测量到电机的数据链路](测量到电机的数据链路.md)。

## 数据接口

车载端约每 50 ms 输出一次测量日志，UART 参数为 115200、8N1。参考板使用 `GPIO03` 输出测量日志：

```text
DIST:200.00, X:100.00, Y:173.21, A:30.00, PD01:0.00, PD02:0.00, PD12:0.00, RSSI:-70
```

上行为格式示例，不是实车采集。DIST、X、Y 单位为 cm，A 单位为 deg，PD 为度制的相位差统计值。PD 与 RSSI 为诊断字段，当前 ESP32 控制不直接使用它们；角度解算确实在 Anchor 使用了 PD12。

```text
X = DIST × sin(A)
Y = DIST × cos(A)
```

DIST 来自空间传播距离，A 来自单基线等效偏角。上述 X/Y 为当前平面控制使用的等效坐标，未补偿佩戴高度差，不应描述为已完成三维矫正的地面坐标。

当前 Anchor 标准安装方式：板卡竖直、Type-C 朝下、天线面朝 FRONT。沿 FRONT 方向看，左侧 A/X 为正，右侧为负。若 Anchor 相对车头有安装偏角，在车辆安装配置中进行统一坐标变换。

## 构建与固件获取

Windows 安装 Keil MDK、ARM Compiler 6 与 CBU5000V210 Device Family Pack。共享目录保留所需的 `.pack` 与 `.lib`；射频库以预编译形式提供。

在 `firmware/uwb_cbu5000v210` 下运行：

```powershell
powershell -ExecutionPolicy Bypass -File .\build_all.ps1
# 自定义安装位置：
powershell -ExecutionPolicy Bypass -File .\build_all.ps1 -KeilUv4 "D:\Keil_v5\UV4\UV4.exe"
```

脚本分别全量构建 Tag / Anchor，检查日志零错误，再写入 `release/` 并计算 SHA256。该目录中的生成固件不进入源码提交。编译通过与射频实测分别记录。

## 调试顺序

先单独验证两端测量，再接 ESP32：固定安装姿态，记录前方、左前与右前三组值；检查距离变化和角度正负。交换两端天线宏可能破坏角度方向，应核对工程目标和硬件布局，而不是在算法多处取反。

Tag 保留既有指示灯行为，蜂鸣器默认关闭。双端功率、时序与天线参数需要配套维护；本次公开版未更改这些射频参数。
