# RDK 可形变全向移动探测机器人 — 项目报告

## 一、项目概述

基于 **RDK X3** 的可形变全向移动探测机器人，面向受限空间巡检/搜救场景。支持**手柄遥控（手动）** 与 **视觉自主（自动）** 双模式，结合 BPU 加速视觉算法实现障碍检测、距离测量、窄通道识别与目标定位。

---

## 二、硬件组成

| 部件 | 型号/规格 | 用途 |
|------|-----------|------|
| 主控板 | RDK X3 | 核心计算 + BPU 推理 |
| 深度相机 | Astra Mini S (Orbbec) | RGB + 深度感知 |
| 下位机 | STM32 | 电机/舵机驱动与串口协议 |
| 手柄 | 飞智 Xbox 兼容手柄 | 手动模式操控 |
| 电机 | 4 路直流电机 | 驱动轮组 |
| PWM 舵机 | 4 路（ID 1~4） | 变形 + 云台 |
| 总线舵机 | 1 路（ID 1） | 摄像头升降 |

串口：`/dev/ttyACM0` @ 1,000,000 baud（协议：0xAA 0x55 头 + CRC8-MAXIM）

---

## 三、软件架构

```
Astra Mini S
    │
    ▼
astra_depth_streamer (C++, 8090端口)
    │  RGB MJPEG + 深度图
    ▼
astra_frame_bridge.py (Python 桥接, 原子写入)
    │  /tmp/rdk_robot_frames/latest_rgb.jpg + depth16.pgm
    ▼
bpu_perception_demo.py (Python, 8091端口)
    │  YOLOv5s_672x672 (BPU), 深度中心距离
    ▼
┌─ manual_mode.py ──── 手动模式（手柄→STM32）
│  rule_controller.py ── 自动模式（视觉决策→STM32）
└─ ai_decision.py ───── 规则引擎 (STOP<0.45m, SLOW<0.90m)
```

**视觉流水线**：Astra → 深度流 (8090) → 帧桥接（原子rename防竞态）→ BPU YOLO 检测 + 深度测距 (8091)

---

## 四、手动模式 (`manual_mode.py v2`)

### 4.1 手柄映射

| 操作 | 按键/摇杆 | 效果 |
|------|-----------|------|
| 油门 | RT 扳机 | 速度 0~MAX_SPEED(0.7) |
| 方向 | 左摇杆 Y | 前/后 |
| 转向 | 左摇杆 X | 左/右（独立于油门，TURN_SCALE=0.35） |
| 形态切换 | A 键 | U 字型 ↔ 一字型 |
| 摄像头升降 | Y 键 | 抬升/放下（含云台回中序列） |
| 模式切换 | B 键 | 手动 ↔ 自动（需 U 型 + 摄像头抬升） |
| 云台旋转 | 右摇杆 X | 摄像头水平旋转（抬升时） |

### 4.2 控制参数

| 参数 | 值 | 说明 |
|------|:---:|------|
| CONTROL_INTERVAL | 0.025s (40Hz) | 电机指令刷新率 |
| MAX_SPEED | 0.7 | 最大电机速度 |
| TURN_SCALE | 0.35 | 转向增益（最大转向 ±0.245） |
| DEADZONE | 0.10 | 摇杆死区 |
| 扳机死区 | 2000 (≈6%) | RT 扳机静区，防噪音误触 |

### 4.3 舵机配置

| 舵机 | PWM ID | U 型 (μs) | 一字型 (μs) |
|------|:------:|:---------:|:----------:|
| 左变形 | 1 | 1600 | 600 |
| 右变形 | 2 | 1550 | 2200 |
| 云台水平 | 3 | — | 500~2500 (默认1500) |
| 俯仰 | 4 | — | 竖直1500 / 水平500 |

| 总线舵机 | ID | 抬升(μs) | 放下(μs) |
|---------|:--:|:--------:|:--------:|
| 摄像头升降 | 1 | 2150 | 1500 |

### 4.4 电机映射（U 字型）

右电机（ID 2, 4）物理反接，代码取负值适配：

| 动作 | 电机1 (ID 0) | 电机2 (ID 1) | 电机3 (ID 2) | 电机4 (ID 3) |
|------|:-----------:|:-----------:|:-----------:|:-----------:|
| 前进 | +speed | -speed(反→前) | +speed | -speed(反→前) |
| 后退 | -speed | +speed(反→后) | -speed | +speed(反→后) |
| **左转** | **+turn** | **+turn**(反→后) | **-turn** | **-turn**(反→前) |
| 右转 | -turn | -turn(反→前) | +turn | +turn(反→后) |

**公式**（`turn = -lx × TURN_SCALE × MAX_SPEED`，左推=正）：
```
m0 = speed + turn
m1 = -speed + turn
m2 = speed - turn
m3 = -speed - turn
```

### 4.5 电机映射（一字型）

电机不反接，全部正值前进：

| 动作 | 电机1 | 电机2 | 电机3 | 电机4 |
|------|:----:|:----:|:----:|:----:|
| 前进 | + | + | + | + |
| 左转 | + | - | - | + |
| 右转 | - | + | + | - |

**公式**：
```
m0 = speed + turn
m1 = speed - turn
m2 = speed - turn
m3 = speed + turn
```

---

## 五、自动模式

`rule_controller.py` + `ai_decision.py`，纯视觉+规则决策直发 STM32，不依赖手柄。

**进入条件**：U 字型 + 摄像头已抬升 → 按 B 键

**规则引擎**（`ai_decision.py`）：

| 条件 | 动作 |
|------|------|
| 深度 < 0.45m 或无数据 | STOP |
| 深度 < 0.90m | SLOW |
| 安全 + 前方可通行 | CLEAR (直行) |
| 探测到障碍偏移 | TURN_LEFT / TURN_RIGHT |

**LLM 语音播报**（`bpu_perception_demo.py`）：
- 模型：`doubao-seed-2-0-mini-260428`（火山引擎）
- 间隔：2 秒（返回即发）
- 风格：120~180 字口语化巡检报告

---

## 九、核心技术分析

### 9.1 视觉感知流水线

感知链路采用**三级解耦架构**，每层独立出错/降级：

**第一层：Astra 硬件采集层（C++）**
- 基于 Astra SDK 直接读取 RGB 与深度帧，`DOWNSCALE=2` 降低分辨率至 320×240（深度）与 640×480（RGB 经 JPEG 压缩）
- 双缓冲 MJPEG 流：RGB 与深度图各自独立线程采集+JPEG 编码（质量 85），经 `pthread_mutex` + `pthread_cond` 同步，通过 HTTP 8090 端口对外提供 `/rgb.mjpg` 与 `/depth.mjpg` 两种格式
- 采集与流推送均运行在独立线程，互不阻塞

**第二层：桥接层（Python）**
- 异步轮询 8090 端口的 MJPEG 流，解析为 OpenCV 矩阵
- 通过**原子写入 + 文件锁**机制（`write_atomic → os.rename`）将 `latest_rgb.jpg` 与 `latest_depth16.pgm` 写入共享目录 `/tmp/rdk_robot_frames/`，避免下游读取到不完整的帧
- 0.3 秒轮询间隔与深度范围截断（400~4000mm）避免无效数据污染

**第三层：BPU 推理层（Python）**
- 同时加载**YOLOv5s 检测模型**与**Cityscapes 语义分割模型**，两个模型并行部署在 BPU 上
- YOLOv5s（输入 672×672）：输出类别 `name`、置信度 `score`、边界框 `bbox`，经深度图映射计算目标距离 `distance_m`
- 语义分割（Cityscapes 19 类）：将场景像素分为 `PASSABLE_CLASSES`（道路、人行道、植被、地形）与 `BLOCKED_CLASSES`（建筑、墙壁、围栏、行人、车辆等），计算 `passable_ratio` 与 `blocked_ratio` 量化可通行程度

**视觉闭环延迟**：从 Astra 采集 → JPEG 编码 → HTTP 传输 → OpenCV 解码 → BPU 推理 → JSON 输出，典型端到端延迟约 250~350ms，主要由 BPU 推理（YOLO + Seg 串行约 150~200ms）主导。

### 9.2 BPU 加速引擎

基于 RDK X3 的 **BPU（Brain Processing Unit）** 伯努利架构，对 YOLOv5s 与 Cityscapes 分割模型做 INT8 量化推理：

- **模型编译**：通过 `hobot_dnn` 加载已量化的 `.bin` 模型，BPU 直接执行卷积层，CPU 仅负责前/后处理
- **算子调度**：`hobot_dnn.pyeasy_dnn` 封装了 BPU 内存管理（`hbSysMem_t` 物理地址映射）与量化反量化（`hbDNNQuantiScale_t` 浮点尺度+零点），开发者无需手动处理 tensor 排布
- **NV12 输入流水线**：RGB 帧先经 `cv2.COLOR_RGB2YUV` 转换为 NV12 格式再送入 BPU，避免多余的像素格式转换
- **后处理全 CPU**：YOLO 的 NMS（非极大值抑制）与分割的 `argmax` 像素分类在 CPU 完成，BPU 仅做卷积推理

**性能收益**：YOLOv5s 在 CPU 上纯浮点推理约 800ms→1.2s/帧，BPU INT8 量化后降至 **35~50ms/帧**，约 **20~30 倍** 加速。Cityscapes 分割模型类似，BPU 推理约 100~150ms。双模型可在约 300ms 的间隔内完成一轮完整感知。

### 9.3 自动模式决策链路

自动模式不依赖手柄，由 `rule_controller.py` 驱动 `ai_decision.py` 规则引擎，按以下层次做出决策：

```
深度图 → 三带分析 → 多阈值比较 → 决策输出 → STM32
            ↓
         YOLO 检测 → 目标距离映射 → 目标级决策
            ↓
       Cityscapes 分割 → 可通行率 → 窄通道识别
```

**深度三带分析**（`ai_decision.py`）：
- 将深度图中心区域水平分割为 **左/中/右** 三带（各占约 18% 宽度）
- 每带计算中值深度，映射到物理距离（单位：米）
- 多个阈值分段：
  - `STOP_DIST = 0.45m` — 紧急制动
  - `SLOW_DIST = 0.90m` — 减速通过
  - `CLEAR_DIST = 1.20m` — 安全直行

**窄通道识别**：
- 当左右带深度均小于 `SIDE_NARROW_DIST(0.65m)` 而中央带大于 `CENTER_PASS_DIST(1.00m)` 时，触发窄通道检测
- 利用中心深度与视场角（ASTRA_HFOV_DEG = 60°）估算物理宽度
- 若宽度 ≥ `MIN_PASSAGE_WIDTH(0.15m)`，建议切换一字形态通过

**侧向不平衡转向**：
- `SIDE_IMBALANCE_RATIO = 2.5`：一侧深度为另一侧 2.5 倍以上时，决策向深侧转向（空间探索策略）

**冷启动防护**：前端数据为空时默认输出 SLOW（减速），避免感知失效时机器人盲目直行。

### 9.4 大模型理解与副驾驶架构

在规则引擎上层，部署了**大模型副驾驶**（`llm_advisor.py`），提供语义理解能力：

**架构约束**：
```
感知数据 → [Safety Rule 硬优先] → LLM 建议 → [Safety Gate] → 最终输出
            ↑  如果 Safety 输出 STOP，LLM 必须 STOP
            ↑  LLM 禁止输出底层控制命令（PWM、串口帧等）
```

**LLM 调用流**：
1. `compact_perception()` 将完整感知数据压缩为 top-5 检测目标（含距离）+ 风险等级
2. `build_messages()` 构造 System + User 消息对，包含输出 JSON Schema
3. `call_llm()` 通过火山引擎 API（`doubao-seed-2-0-mini-260428`）调用，返回 JSON
4. `extract_json()` 做防御性解析（尝试多位置 `raw_decode`），失败则 `make_fallback()` 降级
5. `normalize_advice()` 确保建议落于允许动作集，**不得低于** Safety Rule 的严格程度

**设计哲学**：LLM 只解释场景与给出高层建议，不产生底层控制——体现"副驾驶"定位，Safety Rule 永远是最终权威，即使 LLM 幻觉也不会导致越权控制。

---

## 六、序列脚本

`sequence_move.py` — 预编程动作（不依赖手柄）：

1. 左转 2 秒（速度 0.3）
2. 前进 5 秒（速度 0.4）
3. 停止

---

## 七、启动命令

### 手动模式（手柄优先）
```bash
cd /home/sunrise/rdk_robot
./astra_depth_streamer &                                # 视觉（可选）
python3 -u astra_frame_bridge.py &
python3 -u bpu_perception_demo.py &
python3 -u manual_mode.py                               # 手动控制
```

### 自动模式（视觉自主）
```bash
cd /home/sunrise/rdk_robot
./astra_depth_streamer &
python3 -u astra_frame_bridge.py &
python3 -u bpu_perception_demo.py &
python3 -u rule_controller.py --send                    # --send 实际发串口
```

### 序列动作
```bash
cd /home/sunrise/rdk_robot && python3 -u sequence_move.py
```

---

## 八、关键参数一览

| 项目 | 值 |
|------|:---:|
| 串口设备 | `/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B32012490-if00` |
| 波特率 | 1,000,000 |
| 手柄设备 | `/dev/input/js0` |
| 视觉端口 | 8090（深度流）、8091（BPU YOLO + 面板） |
| 感知 URL | `http://127.0.0.1:8091/result.json` |
| 代码路径 | `/home/sunrise/rdk_robot/` |
| 日志路径 | `/tmp/astra_depth.log`, `/tmp/bpu_perception.log`, `/tmp/manual_mode.log` |
