# Third-party components and provenance

## Project-authored work

The repository's integration code, ESP32 module refactoring, control interfaces, tools and documentation are provided under Apache-2.0. Copyright 2026 jxzzz97. Existing third-party notices remain applicable to their respective components.

## CBU5000V210 SDK

`firmware/uwb_cbu5000v210/shared_sdk` retains the SDK's original Apache-2.0 license file, source headers and bundled component licenses. The UWB application baseline is the author's existing dual-firmware revision `ab5fc6c`; its application implementation, antenna selections and RF configuration are retained in this release.

The SDK includes precompiled vendor libraries (`CBU5000V210_UWB_LIB.lib`, `CBU5000V210_REGULATED_LIB.lib`) and the vendor's Keil device pack. Their implementation sources are not included here. This repository does not describe those binary implementations as newly open-sourced project code. Consult the accompanying vendor notices for the component you use.

| Component | Location | Preserved notice |
| --- | --- | --- |
| CBU5000V210 SDK | `shared_sdk` | `shared_sdk/LICENSE` |
| BLE Host | `shared_sdk/External/BLE_Host` | `LICENSE` and source headers |
| FreeRTOS | `shared_sdk/External/FreeRTOS` | `LICENSE.md` and source headers |
| LibCRC | `shared_sdk/External/LibCRC` | `LICENSE` and source headers |
| Arm CMSIS / startup components | `shared_sdk/Components/ArmCore` | Component source headers |

Paths in this table are relative to `firmware/uwb_cbu5000v210`.

## ESP32 and protocol adapter

The four-state Kalman implementation (`ekf_filter.c/.h`) is retained from the author's previous follower firmware. The public parser preserves its jump-gate and reacquisition policies while tightening numeric parsing. The follow controller is reorganized around explicit physical units and differential kinematics; its numerical behavior is not asserted to match the previous deployed controller.

The reference servo adapter follows the packet layout previously used with the Waveshare DDSM Driver HAT (A). Its project implementation does not bundle the full Waveshare example application. Protocol/device compatibility must be confirmed for each selected actuator.

Arduino ESP32 and PlatformIO dependencies are downloaded by PlatformIO and are not vendored in this repository. Their respective upstream licenses continue to apply.
