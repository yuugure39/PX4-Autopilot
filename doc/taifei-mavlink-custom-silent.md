# MAVLINK_MODE_CUSTOM 全静默方案（备选，当前未采用）

飞控串口直连一块 **MAVLink 智能电池**时使用：电池端只向飞控上报，不需要飞控发给它的任何数据（含心跳）。
本文件记录让该链路完全不发送的做法，需要时按「恢复方法」把改动打回来即可。

- 首次实现：`5a5df653d7`（作者 yuugure39，2026-09-15）
- 已被 `caf906e642` 还原为官方逻辑（2026-10-26），还原原因：其它对端（云台/相机等）需要 custom 模式下的心跳
- 目标平台：CUAV 7-Nano（NuttX），链路为串口

## 当前状态

`taifei` 分支上 `MAVLINK_MODE_CUSTOM` 与官方一致：发送使能打开，默认挂
`HEARTBEAT`(1 Hz)、`STATUSTEXT`(20 Hz)、`COMMAND_LONG`(不限速) 三个流，业务流一条不配。

## 为什么不只是在流表里清空

只把流清掉并不够，官方代码里有三条路径会绕开流表继续发：

1. **模式外的默认流**：`Mavlink::task_main()` 里在 `configure_streams_to_default()` **之前**，
   对「除 IRIDIUM 外的所有模式」无条件挂 `HEARTBEAT` / `STATUSTEXT` / `COMMAND_LONG`。
   其中 `HEARTBEAT` 是 `const_rate()`，明确排除在限速之外（源码注释：*constant rate stream, rate never adjusted*），
   把 `MAV_x_RATE` 压到最低也压不掉。
2. **主循环发送段内的旁路**：`_events.update()`、跨实例转发、命令重试都不查流配置。
   实测流全空后链路上仍有约 17.9 B/s 残留。
3. **接收线程内的发送**：`MavlinkMissionManager::send()` 由**接收线程**周期调用，
   主循环的发送闸门管不到它，任务进度变化时会漏出 `MISSION_CURRENT`（msgid 42）。

## 改动内容（三处）

两处改动都用同一个总闸 `should_transmit()`（`mavlink_main.h`，即 `_transmitting_enabled && ...`）：
`_transmitting_enabled == false` 时，主循环开头的 `if (!should_transmit()) { ...; continue; }` 跳过整段发送逻辑。

| 位置 | 改法 |
|---|---|
| `mavlink_main.cpp`，`_transmitting_enabled = true;` 那段之后（还原后为 2275 附近） | 新增：`if (_mode == MAVLINK_MODE_CUSTOM) { _transmitting_enabled = false; _transmitting_enabled_commanded = false; }` |
| `mavlink_main.cpp`，`/* add default streams depending on mode */`（还原后为 2282 附近） | 条件由 `_mode != MAVLINK_MODE_IRIDIUM` 改为 `_mode != MAVLINK_MODE_IRIDIUM && _mode != MAVLINK_MODE_CUSTOM` |
| `mavlink_mission.cpp`，`MavlinkMissionManager::send()` 开头的守卫（还原后为 497-500） | 守卫追加 `\|\| !_mavlink.should_transmit()` |

副作用：`_transmitting_enabled == false` 后**接收完全不受影响**，电池的 `BATTERY_STATUS` 仍正常进飞控；
该模式也不会被意外重新打开——唯一能恢复 `_transmitting_enabled` 的路径是 `VEHICLE_CMD_CONTROL_HIGH_LATENCY`
命令，而那个分支被 `_mode == MAVLINK_MODE_IRIDIUM` 限定。

## 恢复方法

三种方式都实测可用，按需选一种：

| 方式 | 效果 |
|---|---|
| `git revert caf906e642` | 代码回到静默，同时把 `doc/taifei-changes.md` 里那节也一并恢复（一次性，推荐） |
| `git cherry-pick 5a5df653d7` | 只改这两个源文件，文档需自己同步 |
| `git apply` 本文件末尾的 diff | 只改这两个源文件，适合已改动过上下文、不想带提交历史的场合 |

```bash
cd ~/PX4-Autopilot
git revert caf906e642          # 或：git cherry-pick 5a5df653d7
```

恢复后 `MAVLINK_MODE_CUSTOM` 重新变成全静默；要再切回官方逻辑，把上面那笔提交 revert 回来即可。

## 实测效果（同一块板、同一串口）

| 项目 | 官方逻辑 | 全静默 |
|---|---|---|
| `tx`（`mavlink status`） | 59.7 B/s | 0.0 B/s |
| `tx rate mult` | 0.050（被钳在下限） | 1.000 |
| `TX enabled` | YES | NO |
| 电池端收到的消息 | `HEARTBEAT`(1 Hz)、`MISSION_CURRENT`(1 Hz)、少量 `STATUSTEXT`/`ALTITUDE`/`VFR_HUD` | 无 |

55~60 B/s 这一列另在 SITL 上复现过（20 s 窗口，custom 实例 `mavlink start -u 14560 -o 14561 -m custom`，
收到 20 条 `HEARTBEAT`，共 55.3 B/s）。

## 已知遗留

串口链路启动、进入主循环**之前**会盲发一次 `AUTOPILOT_VERSION`
（`mavlink_main.cpp` 中 `if (get_protocol() == Protocol::SERIAL) { send_autopilot_capabilities(); }`），
不经过 `should_transmit()`，因此每次上电仍有一次突发。当前评估为可接受，未处理。

## 附：完整 diff（`5a5df653d7`，可直接 `git apply` 到当前分支）

```diff
diff --git a/src/modules/mavlink/mavlink_main.cpp b/src/modules/mavlink/mavlink_main.cpp
index 0312a0b6b2..fd35e2811a 100644
--- a/src/modules/mavlink/mavlink_main.cpp
+++ b/src/modules/mavlink/mavlink_main.cpp
@@ -2279,8 +2279,22 @@ Mavlink::task_main(int argc, char *argv[])
 		_transmitting_enabled_commanded = false;
 	}
 
-	/* add default streams depending on mode */
-	if (_mode != MAVLINK_MODE_IRIDIUM) {
+	/* MAVLINK_MODE_CUSTOM is used for links whose peer needs nothing from us (e.g. a smart battery).
+	 * Disable transmitting entirely: should_transmit() gates the whole main loop, so this also covers
+	 * the message paths that bypass the stream configuration above (events, forwarding, command
+	 * retries). Receiving is unaffected. Nothing re-enables it for this mode.
+	 */
+	if (_mode == MAVLINK_MODE_CUSTOM) {
+		_transmitting_enabled = false;
+		_transmitting_enabled_commanded = false;
+	}
+
+	/* add default streams depending on mode.
+	 * MAVLINK_MODE_CUSTOM is excluded as well: it is meant to stream nothing, and a smart battery
+	 * connected on such a link needs no data from the flight controller at all - so also skip the
+	 * otherwise unconditional HEARTBEAT/STATUSTEXT/COMMAND_LONG streams. Receiving is unaffected.
+	 */
+	if (_mode != MAVLINK_MODE_IRIDIUM && _mode != MAVLINK_MODE_CUSTOM) {
 
 		/* HEARTBEAT is constant rate stream, rate never adjusted */
 		configure_stream("HEARTBEAT", 1.0f);
diff --git a/src/modules/mavlink/mavlink_mission.cpp b/src/modules/mavlink/mavlink_mission.cpp
index 822b0144e7..5044d0f6b3 100644
--- a/src/modules/mavlink/mavlink_mission.cpp
+++ b/src/modules/mavlink/mavlink_mission.cpp
@@ -494,8 +494,10 @@ MavlinkMissionManager::send_mission_item_reached(uint16_t seq)
 void
 MavlinkMissionManager::send()
 {
-	// do not send anything over high latency communication
-	if (_mavlink.get_mode() == Mavlink::MAVLINK_MODE_IRIDIUM) {
+	// do not send anything over high latency communication,
+	// nor on a link with transmitting disabled (e.g. MAVLINK_MODE_CUSTOM to a smart battery).
+	// This runs in the receiver task, which bypasses the transmit gate in the mavlink main loop.
+	if (_mavlink.get_mode() == Mavlink::MAVLINK_MODE_IRIDIUM || !_mavlink.should_transmit()) {
 		return;
 	}
 
```
