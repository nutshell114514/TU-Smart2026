# PUSH 修正 PID 参数备份（2026-08-15 置零测试前）

> 用途：为排查"橙球侧（右槽位）推送行驶差、紫球侧（左槽位）正常"而把 PUSH 视觉修正 PID 全部置 0 做 A/B 对照。
> 本文件记录**置零前的原始值**，测试结束后按此表逐项还原即可。

## 1. 主车 task_leader.py

### 1.1 按物体分配的 PUSH PID（实际生效的一组）

`_PUSH_PID_LEADER_BY_OBJ`（第 96~103 行），每项 `(side_kp, side_kd, fwd_kp, fwd_kd)`：

| obj_id | 物体   | 原值                     |
| ------ | ------ | ------------------------ |
| 0      | 不用   | `None`                   |
| 1      | 网球   | `(5.0, 10.0, 3.0, 6.0)`  |
| 2      | 蓝沙包 | `(11.0, 22.0, 5.0, 12.0)` |
| 3      | 红沙包 | `(11.0, 22.0, 5.0, 12.0)` |
| 4      | 白熊   | `(5.0, 10.0, 3.0, 6.0)`  |
| 5      | 棕熊   | `(5.0, 10.0, 3.0, 6.0)`  |

### 1.2 默认兜底 PID（表项为 None 或 obj_id 越界时才用）

| 参数                    | 行号 | 原值 |
| ----------------------- | ---- | ---- |
| `_PUSH_SIDE_KP_LEADER`  | 1867 | `5`  |
| `_PUSH_SIDE_KD_LEADER`  | 1870 | `10` |
| `_PUSH_FWD_KP_LEADER`   | 1873 | `0`  |
| `_PUSH_FWD_KD_LEADER`   | 1877 | `0`  |

## 2. 从车 task_follow.py

### 2.1 按物体分配的 PUSH PID（实际生效的一组）

`_PUSH_PID_FOLLOWER_BY_OBJ`（第 53~60 行），与主车同结构：

| obj_id | 物体   | 原值                      |
| ------ | ------ | ------------------------- |
| 0      | 不用   | `None`                    |
| 1      | 网球   | `(5.0, 10.0, 3.0, 6.0)`   |
| 2      | 蓝沙包 | `(11.0, 22.0, 5.0, 12.0)` |
| 3      | 红沙包 | `(11.0, 22.0, 5.0, 12.0)` |
| 4      | 白熊   | `(5.0, 10.0, 3.0, 6.0)`   |
| 5      | 棕熊   | `(5.0, 10.0, 3.0, 6.0)`   |

### 2.2 默认兜底 PID

| 参数                      | 行号 | 原值 |
| ------------------------- | ---- | ---- |
| `_PUSH_SIDE_KP_FOLLOWER`  | 2381 | `5`  |
| `_PUSH_SIDE_KD_FOLLOWER`  | 2384 | `10` |
| `_PUSH_FWD_KP_FOLLOWER`   | 2387 | `0`  |
| `_PUSH_FWD_KD_FOLLOWER`   | 2391 | `0`  |

### 2.3 球层（V 张开通道）PD

当前 `_PUSH_BALL_CORRECTION_ENABLE = const(0)`，`configure_push` 传入的
`ball_side_kp/ball_side_kd` 已被三元表达式强制为 `0.0`，**这两项本来就不生效**。
一并置零只是为了让"全部 PID = 0"这句话名副其实，不改变任何实际行为。

| 参数                  | 行号 | 原值  |
| --------------------- | ---- | ----- |
| `_PUSH_BALL_SIDE_KP`  | 173  | `1.0` |
| `_PUSH_BALL_SIDE_KD`  | 175  | `2`   |

## 3. 本次未改动的相关项（如需进一步试验再动）

以下不属于"PUSH 修正 PID"，本次保持原值：

- 限幅 / 斜率 / 滤波：`_PUSH_SIDE_MAX_*`、`_PUSH_FWD_MAX_*`、`_PUSH_SIDE_MAX_*_BY_OBJ`、
  `_PUSH_SIDE_SLEW_*`、`_PUSH_FWD_SLEW_*`、`_PUSH_D_LPF_*`。
  PID 全零后这些项对物体环不再有任何影响（`side_raw`/`fwd_raw` 恒为 0）。
- 就位阶段 VDOCK：`_VDOCK_KP = 3.0`、`_VDOCK_OPEN_KP = 4.0`。属于 WAIT_READY，不是 PUSH。
- 无线兜底增益 `_PUSH_WL_GAIN = 1.8`（球层通路，当前同样不生效）。

## 4. 置零后 PUSH 阶段还剩什么

`base._push_step` 里 `side_raw`/`fwd_raw` 恒为 0，`_push_side_cmd`/`_push_fwd_cmd`
经 slew 收敛到 0，因此推送退化为**纯开环**：

- 各车固定基座速度：主车 `_PUSH_SPEED_LEADER = 120`，从车 `_PUSH_SPEED_FOLLOWER`；
- 车头锁定 `_push_lock_yaw`（push_yaw ± 45°）；
- 内偏角 `_push_inward_bias()`。

世界横向锁定早已删除（base.py 第 1982~1983 行注释），所以没有其它隐藏的位置环。
若置零后两侧行驶效果依然一好一坏，即可判定与 PID 无关。
