# taifei 分支改动说明

本文记录飞控开发过程中的改动，基于雷迅7nano v2硬件，后续自制飞控可参考。

- 分支：`taifei`
- 记录时间：2026-09-15
- 主要验证平台：CUAV 7-Nano（STM32H7，NuttX）
- 行号以本文写作时的分支状态为准，后续如有改动请以对应 commit 为准

## 总览

| # | Commit | 类型 | 涉及文件 | 作用 |
|---|---|---|---|---|
| 1 | `eec0da0d05` | feat | `src/modules/mavlink/streams/OPEN_DRONE_ID_BASIC_ID.hpp` | `uas_id` 改为按 GB 46860-2025 输出 20 位 ID |
| 2 | `4746115da3` | fix | `src/modules/mavlink/streams/OPEN_DRONE_ID_BASIC_ID.hpp` | 修复 SITL/POSIX 链接失败 |

---

## 改动一：Remote ID 的 `uas_id` 改为 GB 46860-2025 格式

**Commit**：`eec0da0d05`

### 改动原因

`OPEN_DRONE_ID_BASIC_ID` 消息中的 `uas_id` 字段（`uint8_t[20]`）原先由
`board_get_px4_guid_formated()` 填充，得到的是**截断后的 PX4 GUID 十六进制字符串**。
该函数在 `size = 20` 时会被强制成 19，实际只写出 18 个十六进制字符加结尾 `\0`，
既不符合 ANSI/CTA-2063，也不符合 GB 46860-2025 对无人机唯一标识的要求。
源码中原本就留有 `TODO: MAV_ODID_ID_TYPE_SERIAL_NUMBER needs to be ANSI/CTA-2063 format` 的提示。

### 改动文件与行号

文件：`src/modules/mavlink/streams/OPEN_DRONE_ID_BASIC_ID.hpp`

| 行号 | 内容 |
|---|---|
| 37-40 | 新增 include：`<stdio.h>`、`<string.h>`、`<px4_platform_common/board_common.h>` |
| 85-89 | 格式说明注释 |
| 90-91 | 制造商码 / 产品型号码常量 `kGbidManufacturerCode`、`kGbidModelCode`（当前为占位值 `TFHK` / `TEST`） |
| 93 | 序列号哈希初值（FNV-1a offset basis） |
| 95-115 | 序列号派生：板载 GUID 的 FNV-1a 哈希（含 SITL 回退分支，见改动二） |
| 117-120 | `snprintf` 拼装 20 位字符串 |
| 122 | 拷入 `msg.uas_id` |

### 改动作用

`uas_id` 现在输出 GB 46860-2025 规定的 **4 位制造商码 + 4 位产品型号码 + 12 位序列号 = 20 位** ASCII，
正好填满 20 字节字段（不带结尾 `\0`），例如 `TFHKTEST711917160103`。

- 制造商码与型号码用于标识产品，由常量给出；
- 12 位序列号由**板载 18 字节 PX4 GUID 经 FNV-1a 哈希后对 `10^12` 取模**得到，
  因此每台设备不同，满足唯一性要求；字符集为大写字母与数字，兼容
  `MAV_ODID_ID_TYPE_SERIAL_NUMBER` 的要求；
- `id_type` 保持 `MAV_ODID_ID_TYPE_SERIAL_NUMBER` 不变（MAVLink 的 `MAV_ODID_ID_TYPE`
  枚举中没有与 GB 46860-2025 一一对应的类型，当前四种取值中该值最贴近）。

### 验证

烧录到 CUAV 7-Nano 后，QGC 的 MAVLink Inspector 中 `OPEN_DRONE_ID_BASIC_ID` 的 `uas_id`
字段为 20 个 ASCII 字节，内容为 `TFHKTEST711917160103`。

该值可由该板 `ver all` 输出的 `PX4GUID`（`0006000000003037363035325111002d0029`）独立复算得到，
算法实现与文档中的描述一致。

> 注意：`kGbidManufacturerCode` 与 `kGbidModelCode` 目前是占位值。正式使用前需替换为
> 实际注册的制造商码与产品型号码；替换后设备广播的 ID 会随之改变。

---

## 改动二：修复 SITL/POSIX 链接失败

**Commit**：`4746115da3`

### 改动原因

改动一引入了对 `board_get_px4_guid()` 的调用，但该函数**仅在 NuttX 平台下实现**
（`platforms/nuttx/src/px4/*/version/board_identity.c`）。POSIX/SITL 目标在
`platforms/posix/include/system_config.h` 中定义了 `BOARD_HAS_NO_UUID`，
其板级标识实现来自 `platforms/common/board_identity.c`（编译进 `libpx4_platform.a`），
只提供 `board_get_px4_guid_formated()`、`board_get_uuid32()`、`board_get_uuid32_formated()`，
**没有** `board_get_px4_guid()`。

因此 `make px4_sitl gz_x500` 在链接阶段失败：

```
OPEN_DRONE_ID_BASIC_ID.hpp:94: error: undefined reference to 'board_get_px4_guid'
collect2: error: ld returned 1 exit status
```

### 改动文件与行号

文件：`src/modules/mavlink/streams/OPEN_DRONE_ID_BASIC_ID.hpp`

| 行号 | 内容 |
|---|---|
| 95 | `#ifndef BOARD_HAS_NO_UUID` |
| 96-102 | 原 NuttX 路径不变：`board_get_px4_guid()` + 对 GUID 逐字节哈希 |
| 104-113 | `#else` 分支：改用所有平台都提供的 `board_get_uuid32()` 作为哈希输入 |
| 115 | `#endif` |

### 改动作用

采用 PX4 自身的既有惯用法（`UTM_GLOBAL_POSITION.hpp` 与 `send_autopilot_capabilities()` 均如此处理）：

- **NuttX 路径行为完全不变**，设备序列号不变。NuttX 目标不会定义 `BOARD_HAS_NO_UUID`，
  因此走的是 `#ifndef` 分支，哈希的输入、算法、常量均与改动一一致；
- **SITL/POSIX 有了确定性回退**：用 `board_get_uuid32()` 的 16 字节作为哈希输入，
  仿真下也能得到稳定的 `uas_id`，不再链接失败。

### 验证

| 目标 | 命令 | 结果 |
|---|---|---|
| SITL | `ninja bin/px4`（`build/px4_sitl_default`） | 通过 |
| NuttX | `ninja cuav_7-nano_default.px4` | 通过 |

固件体积由 1749099 变为 1749111 字节（+12），系 `serial_hash` 声明位置上移导致的代码布局变化；
哈希算法未变，因此设备序列号仍为 `TFHKTEST711917160103`，无需重新烧板验证。

---

## 附：相关但未改动的项

以下问题在排查过程中确认，但按当前需求**保持原样**，记录备查：

- **`AUTOPILOT_VERSION` 的 `uid2`**：仍为 18 字节原始 `px4_guid`，未改为 GB 格式
  （`mavlink_main.cpp` 的 `send_autopilot_capabilities()`）。该字段语义是硬件 UID，
  且仅 18 字节，装不下 20 位 ID。
- **DroneCAN Remote ID**：`src/drivers/uavcan/remoteid.cpp` 仍以 `board_get_px4_guid_formated()`
  填充 `uas_id`。如后续使用 DroneCAN 的 ODID 广播，需同样改为 GB 格式。
- **`UTM_GLOBAL_POSITION`**：其 `uas_id` 是 16 字节二进制 UUID，字段语义与 ODID 不同，**不应修改**。
