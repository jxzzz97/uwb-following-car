# Shared SDK Local Changes

本目录来自 CBU5000V210 SDK。为了让 Tag 和 Anchor 真正共用同一份源码，只做了一项本地适配：

- 文件：`Components/Midlayer/UwbFramework/CB_uwbframework.c`
- 新增编译宏：`UWB_ANTENNA_TYPE_CONFIG`
- Tag 工程选择：`DEF_ANTENNA_TYPE_TRIANGLE_UP`
- Anchor 工程选择：`DEF_ANTENNA_TYPE_TRIANGLE_DOWN`

除该天线布局选择外，`Boards`、`Components`、`External`、`Libs`、`Docs` 和 `Tools` 均保留 SDK 内容。

本地适配已经通过双端 Keil 全量编译，并确认生成的两份 HEX 分别与整理前原工程完全一致。
