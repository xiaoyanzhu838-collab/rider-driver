# 11 / 21 轮毂节点驱动说明

日期：2026-04-17

工作区：
- `D:\workspace\code\rider_driver`

前提：
- `UART2` 总线当前实测在线节点：`11 / 12 / 21 / 22`
- `12 / 22` 是会刮机身的展开舵机，因此每次主动测试 `11 / 21` 前，都先把：
  - `12 -> 578`
  - `22 -> 461`

## 结论

`11 / 21` 在总线协议层面仍然是 `Legacy DXL-like` 节点，但它们的机械语义更像“被 DXL 风格寄存器包装的 FOC 轮毂电机”，而不是普通位置舵机。

当前最符合实测的驱动模型是：

1. `goal_position (0x1E)` 不是一个单独就能稳定驱动轮子的“位置目标”。
2. `goal_speed (0x20)` 对 `11 / 21` 有决定性影响。
3. 当 `goal_speed` 很低时：
   - 即便给了一个很小的 `goal_position` 偏移，轮子也基本不动。
4. 当 `goal_speed` 提高到中高值时：
   - 轮子开始持续旋转；
   - `present position (0x24)` 表现为 `0..1023` 的模位置反馈；
   - `present speed (0x26)` 会跟着变大；
   - `moving (0x2E)` 仍然长期为 `0`，不可靠。

所以就现阶段来说，`11 / 21` 更适合被理解成：

- `speed` 决定轮子的实际转动强度
- `goal_position` 更像一个触发/锁存配套量，而不是普通舵机那种“到某个角度就停住”

补充后的方向结论：

- `goal_position` 的正负方向，不决定 `11 / 21` 的实际转向。
- 经典 Dynamixel 轮模式常见的 `goal_speed` 方向位（`0x0400`）在这里也没有表现成“反向”。
- 对这两个节点，更像是：
  - 始终沿内部固定正向做模位置推进
  - `goal_speed` 的数值大小决定推进速度
  - `goal_position` 只是参与内部状态机，而不是直接决定正反转

## 实测摘要

### 1. 保持当前位置 + 低速

对两个轮子都做了：
- 读取当前位置
- `torque on`
- 写 `goal = current_position`
- 写 `speed = 10`

结果：
- `11` 保持在 `852`
- `21` 保持在 `180`
- 10 个采样窗口内 `total_forward = 0`

说明：
- 当前位置保持是成立的
- 并不存在“只要一上扭矩就自己持续转”的现象

### 2. 只改 speed，不改 goal

对 `11 / 21` 分别测试了：
- `speed = 10`
- `speed = 40`
- `speed = 100`

结果很一致：
- `speed = 10`：基本不动
- `speed = 40`：开始持续转动
- `speed = 100`：转得明显更快

代表性摘要：
- `ID11`
  - `speed-only 10` -> `total_forward = 0`
  - `speed-only 40` -> `total_forward = 1021`
  - `speed-only 100` -> `total_forward = 4361`
- `ID21`
  - `speed-only 10` -> `total_forward = 0`
  - `speed-only 40` -> `total_forward = 876`
  - `speed-only 100` -> `total_forward = 4211`

说明：
- 对 `11 / 21`，单独写 `goal_speed` 就能直接改变轮子行为
- 这不是普通位置舵机的典型表现

### 3. 小步 goal + 不同 speed

对两个轮子分别做：
- `delta = +8`
- `speed = 10 / 40 / 100`

结果：
- `speed = 10`：仍然不动
- `speed = 40`：开始持续转动
- `speed = 100`：转得明显更快

代表性摘要：
- `ID11`
  - `goal+8, speed=10` -> `total_forward = 0`
  - `goal+8, speed=40` -> `total_forward = 1009`
  - `goal+8, speed=100` -> `total_forward = 4361`
- `ID21`
  - `goal+8, speed=10` -> `total_forward = 0`
  - `goal+8, speed=40` -> `total_forward = 892`
  - `goal+8, speed=100` -> `total_forward = 4261`

说明：
- 真正决定“会不会转、转多快”的仍然是 `goal_speed`
- `goal_position` 在这里不像 12 / 22 那样具备经典位置舵机语义

### 4. 方向现象

这轮新增了更明确的方向实验。

#### 4.1 `goal_position` 正负大偏移

分别测试了：
- `delta = +128, speed = 40`
- `delta = -128, speed = 40`

并且使用模 1024 最短路计算，确保：
- `+128` 的理论最短方向是正向
- `-128` 的理论最短方向是反向

结果：
- `ID11`
  - `base=821 -> goal=949, shortest=+128`
  - `TRACE ... total_forward = 788`
  - `base=934 -> goal=806, shortest=-128`
  - `TRACE ... total_forward = 789`
- `ID21`
  - `base=322 -> goal=450, shortest=+128`
  - `TRACE ... total_forward = 677`
  - `base=284 -> goal=156, shortest=-128`
  - `TRACE ... total_forward = 701`

说明：
- 即便理论最短方向是负向，两个轮子仍然继续按“正向绕回”的方式推进
- `goal_position` 的符号并不决定轮子正反转

#### 4.2 经典 DXL `goal_speed` 方向位

测试了两组速度写法：
- 正常低速：`40`
- 带经典方向位：`0x0400 | 40 = 1064`

结果：
- `ID11`
  - `speed=40` -> `total_forward = 759`
  - `speed=1064` -> `total_forward = 2601`
  - `present speed` 变成了约 `695~717`
  - 没有出现反向，反而变成了更快的同向推进
- `ID21`
  - `speed=40` -> `total_forward = 701`
  - `speed=1064` 的首个采样：
    - `pos=883`
    - `speed_raw=659`
    - `load_raw=58`
  - 同样没有出现反向迹象，而是直接进入了更高速的正向推进

说明：
- 对 `11 / 21`，`0x0400` 这类经典方向位并没有表现成反向控制
- 它更像只是把速度数值整体抬高了

当前结论：
- 方向并不是简单由 `goal_position` 相对当前位置的正负号决定
- 也不是经典 DXL wheel-mode 的 `0x0400` 方向位决定
- `11 / 21` 很可能内部还有额外方向/闭环策略，被 FOC 驱动板封装掉了

### 5. 速度死区与停转语义

这轮又补了两类更贴近接口落地的测试：
- `goal_speed` 死区扫描
- `speed = 0` 与 `torque off` 的停转对比

#### 5.1 `11` 的死区

- `speed = 0 / 1 / 5 / 10 / 15 / 20`
  - `total_forward = 0`
- `speed = 25`
  - `total_forward = 12`
  - 只是极轻微挪动
- `speed = 30`
  - `total_forward = 232`
  - 已经进入可见的稳定转动
- `speed = 40`
  - `total_forward = 529`
- `speed = 60`
  - `total_forward = 1139`

结论：
- `11` 的起转阈值在 `25~30` 之间
- `30` 可以视为较稳妥的最小有效转动档位

#### 5.2 `21` 的死区

- `speed = 0 / 1 / 5 / 10 / 15 / 20`
  - `total_forward = 0`
- `speed = 25`
  - `total_forward = 30`
  - 只有轻微 creeping
- `speed = 30`
  - `total_forward = 118`
  - 已开始稳定起转
- `speed = 40`
  - `total_forward = 467`
- `speed = 60`
  - `total_forward = 1060`

结论：
- `21` 的起转阈值同样在 `25~30` 之间
- `30` 也是较稳妥的最小有效转动档位

#### 5.3 停转方式

先让轮子以 `speed = 40` 转动，再测试两种停法：

- `speed = 0`
  - `ID11`: `TRACE WHEEL_STOP_SPEED0 SUMMARY ... total_forward = 0`
  - `ID21`: `TRACE WHEEL_STOP_SPEED0 SUMMARY ... total_forward = 0`
- `torque off`
  - `ID11`: `TRACE WHEEL_STOP_TORQUE_OFF SUMMARY ... total_forward = 0`
  - `ID21`: `TRACE WHEEL_STOP_TORQUE_OFF SUMMARY ... total_forward = 0`

结论：
- 这两个节点上，`speed = 0` 本身就足以把轮子停住
- `torque off` 也能停，但对后续接口来说，`speed = 0` 更适合当作常规停转命令
- `torque off` 更适合作为“完全释放/断使能”语义，而不是日常调速闭环里的主停法

## 当前可落地的理解

如果只是“把 11 / 21 用起来”，当前最务实的理解是：

- 把它们当成“速度主导型轮毂节点”
- 不要把它们当普通绝对位置舵机
- `speed <= 20` 基本静止
- `speed = 25` 处在 creeping 边缘
- `speed = 30` 是目前更合适的最小有效转动档
- `speed = 40` 已经能稳定低速转
- `speed = 100` 已经明显更快
- 日常停转优先用 `speed = 0`
- 如需彻底释放，再用 `torque off`

## 风险提醒

- 不建议去改 `angle limit / ID / baud / reset`
- 不建议在地面大幅度连续试轮子，除非先确认机体姿态安全
- `moving` 反馈当前对 `11 / 21` 不可靠，不能当作“轮子是否在动”的主判断依据

## 相关日志

- `D:\workspace\code\rider_driver\logs\motor_probe_wheel_drive_2026-04-17.log`
- `D:\workspace\code\rider_driver\logs\motor_probe_wheel_drive_2026-04-17_raw.log`
- `D:\workspace\code\rider_driver\logs\motor_probe_wheel_drive_2026-04-17_events.jsonl`
- `D:\workspace\code\rider_driver\logs\motor_probe_wheel_direction_2026-04-17.log`
- `D:\workspace\code\rider_driver\logs\motor_probe_wheel_direction_2026-04-17_raw.log`
- `D:\workspace\code\rider_driver\logs\motor_probe_wheel_direction_2026-04-17_events.jsonl`
- `D:\workspace\code\rider_driver\logs\motor_probe_wheel_control_2026-04-17.log`
- `D:\workspace\code\rider_driver\logs\motor_probe_wheel_control_2026-04-17_part2.log`
- `D:\workspace\code\rider_driver\logs\motor_probe_wheel_midrange_2026-04-17.log`

## 当前固件状态

主动探测任务代码已经保留在仓库里，但板子已刷回稳定启动版：

- 上电仅拉起原本的电池检测和 BLE
- 不会再自动跑 `11 / 21` 轮子探测
