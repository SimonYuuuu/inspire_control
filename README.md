# hand_teleop

USB 摄像头 → 手部关键点 → 手指关节角 / 手掌姿态。
这是遥操作 Inspire 灵巧手的第一阶段：只做视觉感知，输出结构化的手势数据。

## 组成

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

## 下一步（接 Inspire 灵巧手）

Inspire 五指手是 6 自由度（小指 / 无名指 / 中指 / 食指 / 拇指弯曲 / 拇指旋转），
驱动值 0~1000。衔接点很清楚：

1. `curls` 的 5 个值 → 对应 5 个弯曲自由度（注意 Inspire 的 0 = 闭合、1000 = 张开，方向要反一下）。
2. `spreads["thumb_index"]` → 拇指旋转自由度。
3. 加一层限速 / 死区，再通过 Modbus-RTU（串口）或厂家 SDK 下发。

实现这层时建议新增 `inspire_hand.py`（协议与关节映射）+ `teleop.py`（闭环循环），
保持本目录现有模块不动。

## 注意

- 本机摄像头：Logitech BRIO，主摄为 `/dev/video0`（`--list` 可复查）。
- anaconda 的 python3 建 venv 会在装包时报 `sre_compile` 错误，`setup.sh` 需用系统解释器：
  `PY=/usr/bin/python3 ./setup.sh`。
- 装的是 mediapipe 1.x，旧的 `mp.solutions.hands` 已被移除，本项目用的是 Tasks API。
