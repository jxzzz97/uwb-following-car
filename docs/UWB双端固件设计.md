# CBU5000V210 UWB 双端固件设计

## 工程组织与角色

`tag_firmware` 是 initiator，安装在随身端；`anchor_firmware` 是 responder，安装在车端。两端共享 `shared_sdk`，各自保留 App、Keil 工程和链接配置。当前版本继承已有双端固件的业务实现。

共享 framework 通过编译宏 `UWB_ANTENNA_TYPE_CONFIG` 选择天线布局：Tag 使用 `DEF_ANTENNA_TYPE_TRIANGLE_UP`，Anchor 使用 `DEF_ANTENNA_TYPE_TRIANGLE_DOWN`。该宏是实际硬件布局的选择，不是用于随意交换左右方向的开关。

## 数据接口

车载端约每 50 ms 输出一次测量日志，UART 参数为 115200、8N1。参考板使用 `GPIO03` 输出测量日志：

```text
DIST:200.00, X:100.00, Y:173.21, A:30.00, PD01:0.00, PD02:0.00, PD12:0.00, RSSI:-70
```

上行为格式示例，不是实车采集。DIST、X、Y 单位为 cm，A 单位为 deg。PD 与 RSSI 为诊断字段，当前 ESP32 控制不使用它们。

```text
X = DIST × sin(A)
Y = DIST × cos(A)
```

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
