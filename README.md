# hand_teleop

USB 摄像头 → 手部关键点 → 手指关节角 / 手掌姿态 → Inspire 灵巧手。

上半部分（仓库根目录）只做视觉感知，输出结构化手势数据；
下半部分（[hand/](hand)）把它变成灵巧手的关节指令，带安全限制和急停。

## 组成（视觉端）

| 文件 | 作用 |
| --- | --- |
| [camera.py](camera.py) | 摄像头枚举（带设备名）/ 打开 / `StreamReader` 同步读取与限速 |
| [hand_tracker.py](hand_tracker.py) | MediaPipe Tasks `HandLandmarker`（VIDEO 模式）封装，带 EMA 平滑 |
| [hand_pose.py](hand_pose.py) | 纯几何计算：关节角、curl 归一化、指间张角、手掌坐标系 |
| [visualize.py](visualize.py) | 骨架 / 手掌三轴 / curl 条形图 / FPS / 延迟叠加 |
| [run_hand_tracking.py](run_hand_tracking.py) | 实时主程序（窗口预览或 headless JSON 流）|
| [selftest.py](selftest.py) | 不需要摄像头的几何自检 |

模型文件 `models/hand_landmarker.task` 首次运行时自动下载。

## 安装

```bash
cd ~/Dimos/hand_teleop
./setup.sh                       # 建 .venv 并装 mediapipe / opencv / numpy
```

## 运行

```bash
./.venv/bin/python run_hand_tracking.py --list          # 列出可用摄像头
./.venv/bin/python run_hand_tracking.py --device 0      # 实时预览，q / ESC 退出
./.venv/bin/python run_hand_tracking.py --device 0 --headless --log poses.jsonl
```

常用参数：`--max-hands`、`--smoothing 0.7`、`--no-flip`、`--width/--height`、`--max-fps`。

`--list` 会直接列出设备名，不用猜哪个 `/dev/videoN` 是真的摄像头：

```
index  resolution   fps    name
0      640x480      30.0   Logitech BRIO
2      340x340      30.0   Logitech BRIO
```

UVC 摄像头会注册多个节点（彩色流 + 红外/元数据），上例中 `0` 是主摄，`2` 是 340x340 的红外节点。
设备名读自 `/sys/class/video4linux/videoN/name`。

## 实时性与帧率

推理比采集快得多，所以采用**单线程同步读取**：每读到一帧立即推理，输入速率和输出速率天然一致，
驱动队列不会堆积，也就不会滞后。不需要抓帧线程或丢帧逻辑。

本机实测（CPU，Logitech BRIO 640x480）：

| 环节 | 耗时 |
| --- | --- |
| 摄像头等待一帧 | 27.5 ms |
| 手部推理 | **8.5 ms** |
| 管线吸吐 | 27.7 FPS |
| 采帧→姿态延迟 | 中位 12.4 ms，p95 23 ms |

帧率瓶颈在摄像头（非推理）。若不需要这么快，用 `--max-fps` 降输入帧率：

```bash
./.venv/bin/python run_hand_tracking.py --device 0 --max-fps 10
```

实测 `--max-fps 10` → 9.3 FPS，`--max-fps 5` → 4.7 FPS，而**延迟始终保持在 12 ms 左右**。
限速采用按帧计数丢弃（`grab()` 不解码），而不是按时钟等待：后者每轮会多等一个摄像头周期，
实测会把 10 FPS 误伤成 6.2 FPS。

> 坑：不要设 `CAP_PROP_BUFFERSIZE=1`。在 UVC 摄像头上它会把交付帧率恰好减半（27.8 → 13.8 FPS），
> 而它本来就是多余的——消费端从不落后。

## 输出数据格式

每帧每只手一条记录：

```json
{
  "t": 12345.678,
  "latency_ms": 12.4,
  "hands": [{
    "handedness": "Right",
    "score": 0.99,
    "joint_angles": {"index": [45.6, 47.5, 66.8], "...": []},
    "curls":   {"thumb": 0.21, "index": 0.62, "middle": 0.63, "ring": 0.59, "pinky": 0.68},
    "spreads": {"thumb_index": 31.2, "index_middle": 9.8, "middle_ring": 7.1, "ring_pinky": 10.4},
    "palm_euler":  [0, 0, 0],
    "palm_normal": [0, 0, 0],
    "palm_center": [0, 0, 0]
  }]
}
```

- `curls` 为 0（张开）~ 1（握紧），由三个关节角之和归一化，归一化区间在
  [hand_pose.py](hand_pose.py) 的 `CURL_RANGE_DEG` 里，可按操作者手型标定。
- `palm_*` 来自手掌坐标系：x 沿掌面横向（小指→食指），y 指向手指，z 为掌心法向。
- 手腕姿态用的是 MediaPipe world landmarks（以手部中心为原点的近似米制坐标），
  只含朝向信息，不含手相对相机的绝对平移。

## 下发到 Inspire 灵巧手（`hand/`）

[hand/](hand) 把上面的姿态输出接到 Inspire RH56DFTP 的 6 个自由度
（小指 / 无名指 / 中指 / 食指 / 拇指弯曲 / 拇指旋转），驱动值 0~1000。

| 文件 | 作用 |
| --- | --- |
| [hand/registers.py](hand/registers.py) | 寄存器地址表（只收录在真机上验证过的） |
| [hand/client.py](hand/client.py) | 裸 socket 的 Modbus TCP 客户端 + `SimulatedHand` |
| [hand/mapping.py](hand/mapping.py) | `HandPose` → 每个自由度的「伸展度」0~1 |
| [hand/safety.py](hand/safety.py) | 行程限幅 / 力保护 / 变化率限制 / 死区 |
| [hand/controller.py](hand/controller.py) | 独立控制线程、状态机、看门狗、急停 |
| [hand/teleop.py](hand/teleop.py) | 主程序（预览窗口 + 按键急停） |
| [hand/selftest.py](hand/selftest.py) | 不需要硬件的映射与安全逻辑自检 |

```bash
./.venv/bin/python -m hand.selftest                        # 纯逻辑自检
./.venv/bin/python -m hand.teleop --dry-run                # 不碰硬件，先看映射对不对
./.venv/bin/python -m hand.teleop --host 192.168.123.211   # 实机
```

**默认是不跟随的**。启动后手不会动，按空格才开始跟随，这样插电启动的瞬间不会突然动作。

| 按键 | 作用 |
| --- | --- |
| `空格` | 跟随 / 暂停 |
| `ESC` | 急停（闭锁） |
| `c` | 解除闭锁，回到不跟随状态 |
| `q` | 退出（退出前一定会先急停） |

### 方向约定

两边是反的，映射层负责掰过来：`curls` 0 = 张开，而角度寄存器 1000 = 伸直。
拇指旋转取自 `spreads["thumb_index"]`，默认把 15°~55° 的外展映射到 0~1000；
手型不同可以用 `--thumb-rot-range 20:50` 调，转反了就加 `--invert-thumb-rot`。
先用 `--dry-run` 对着屏幕上的数值确认，再上实机。

### 安全限制

三道闸串联，在 [hand/safety.py](hand/safety.py) 里：

1. **行程限幅** — 任何情况下都不会发出 0~1000 以外的值。可以再收窄：
   `--limit thumb_bend=600:1000 --limit index=500:1000`（可重复）。
2. **力保护** — 轮询 `FORCE_ACT`，某个自由度的夹持力超过 `--force-limit`（默认 1500 g）
   时，该自由度**只允许张开、不允许继续闭合**，卡住的手指不会越压越死。
   力值是有符号 int16（单位克），客户端已经按有符号解析。
3. **变化率限制** — `--max-rate`（默认 900 单位/秒，全行程约 1.1 秒）。
   视觉跳变只会变成缓慢漂移，不会变成全速甩动。

再加一个 `--deadband`（默认 6）吃掉手抖，避免电机在原地嗡嗡响。

### 急停

软件急停**不是安全级急停**。网络断了、进程被 kill 了、PC 死机了，它都不起作用。
真正的急停只有一个：**把 24 V 断掉**，所以电源开关要放在手边。

在软件能管到的范围内，这几条都会触发闭锁式停止：

| 触发源 | 场景 |
| --- | --- |
| `ESC` 按键 | 操作者看到不对劲 |
| 看门狗 | 超过 `--watchdog-ms`（默认 300 ms）没有检测到手 —— 手出画、遮挡、跟踪崩了 |
| 相机断流 | USB 被拔掉 |
| `SIGINT` / `SIGTERM` | Ctrl-C 或被 kill |
| Modbus 异常 | 断线进 `fault`，自动重连，恢复后回到**不跟随**状态，不会自己接着动 |

停止动作由 `--stop-action` 决定：

- `hold`（默认）— 先读回 `ANGLE_ACT`，再把实际角度**写回**设定寄存器。
  这样是在手指**物理所在的位置**停住，而不是停在「上一个目标值」（目标往往已经跑到前面去了）。
- `open` — 张开到行程上限，松手放开抓着的东西。

闭锁之后会忽略一切跟踪输入，必须按 `c` 显式解除，解除后也只回到不跟随状态，要再按空格才继续。

### 实时性

控制线程独占 Modbus socket，视觉循环只往里扔最新结果（`submit()`），从不等待网络。
这样 Modbus 卡一下不会拖慢跟踪，而且视觉循环死掉时看门狗照样在跑。

| 环节 | 实测 |
| --- | --- |
| Modbus 往返（读/写） | 中位 **0.59 ms**，p95 5.9 ms |
| 控制线程周期 | 请求 50 Hz → 实测 49.6 Hz（20/100 Hz 同样准） |
| 采帧→姿态 | 中位 12.4 ms |

端到端大约 **25~35 ms**，瓶颈在摄像头，不在网络。

50 Hz 连续写入 20 秒（约 1000 次）实测灵巧手全程响应正常，不会僵死——前提是每次都把 ACK 读掉。

> 实测：实际角度会比设定值低几个单位（拇指旋转约 13 个单位，其余 2~5 个），是伺服稳态偏差。
> 默认 `--deadband 6` 与之同量级，所以死区不会和这个偏差来回打架。

### 固件注意事项

- **每次写入返回的 12 字节 ACK 必须读掉。** 不读的话，字节堆在灵巧手的 TCP 发送缓冲区里，
  内核流控会阻塞它的 `send()`，它的主循环就卡住不再响应指令——而你这边 `sendall()` 一直是成功的。
  表现就是「动几秒之后突然僵住」。[hand/client.py](hand/client.py) 里每次写都排空 ACK。
- **断开后要等 ~300 ms 再重连**，否则连上之后会立刻掉线。
- 本机灵巧手实际 IP 是 `192.168.123.211:6000`，不是文档里的默认值；网口需配
  `192.168.123.100/24`。
- 网传 RH56E2 资料里的错误码 / 状态寄存器（1606 / 1612）在这台手上按 16 位读出来是错位的，
  所以没有采用，也不建议直接照抄。


## 注意

- 本机摄像头：Logitech BRIO，主摄为 `/dev/video0`（`--list` 可复查）。
- anaconda 的 python3 建 venv 会在装包时报 `sre_compile` 错误，`setup.sh` 需用系统解释器：
  `PY=/usr/bin/python3 ./setup.sh`。
- 装的是 mediapipe 1.x，旧的 `mp.solutions.hands` 已被移除，本项目用的是 Tasks API。
