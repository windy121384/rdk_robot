# RDK X3 Morphing Inspection Robot Notes

## Project Goal

Build a morphing omnidirectional inspection/search robot for constrained scenes such as nuclear facility inspection, industrial pipeline detection, and post-disaster narrow-space rescue.

RDK X3 is the upper computer. It handles camera acquisition, visual AI inference, motion decisions, morphing decisions, and serial command output to an STM32 lower controller.

## Hardware

- Upper computer: RDK X3
- Lower controller: STM32 control board
- Camera: Orbbec Astra Mini S depth camera
- Gamepad: Flydigi Direwolf in XInput mode, `/dev/input/js0`
- Serial to STM32:
  - Device seen as `/dev/ttyACM0`
  - Stable path seen as `/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B32012490-if00`
  - Baudrate: `1000000`, `8N1`

## Actuators

- 4 motors for chassis movement
- 4 PWM servos:
  - 2 servos for body morphing: U-shape <-> line shape
  - 1 servo for camera pan-tilt yaw/pan
  - 1 servo for changing camera posture from vertical to horizontal
- 1 bus servo:
  - Raises the camera, because the camera initially lies inside the robot

## Camera Status

Astra SDK is usable.

Verified:

- RGB capture works
- Depth capture works
- IR official sample works
- Official Body Tracking sample does not work without Orbbec Body Tracking license
- RGB image appeared mirrored; horizontal flip can correct it

Existing generated examples were under:

- `/root/.openclaw/workspace/astra_frames/`
- Some images were copied to `/home/sunrise/Desktop/`

## Control Behavior

Initial state:

- Shape: U-shape
- Drive mode: differential
- Camera: down/stowed

Gamepad mapping:

- Left stick:
  - `axis 0`: left/right, left negative, right positive
  - `axis 1`: forward/back, up negative, down positive
- Right stick:
  - `axis 3`: left/right, left negative, right positive
  - `axis 4`: up/down, but only right-stick X is used for camera pan
- Buttons:
  - `A = button 0`: toggle U-shape / line shape
  - `B = button 1`: currently not assigned in final logic
  - `X = button 2`: toggle mecanum / differential drive, initial differential
  - `Y = button 3`: raise/stow camera
- Triggers:
  - `LT = axis 2`
  - `RT = axis 5`: motor speed control

Manual control rules:

- When robot is U-shaped:
  - Left stick controls forward/back/left/right in mecanum mode
  - In differential mode, left stick Y controls forward/back and left stick X controls yaw
- `X` toggles mecanum vs differential chassis mode
- `Y` raises the camera
- After camera is raised:
  - Right stick X controls camera pan only
  - The PWM servo that changes camera posture from vertical to horizontal should actuate directly
- `RT` controls motor speed
- `A` toggles line shape vs U-shape
- In line shape, keep movement conservative: forward/back plus yaw, no lateral translation

## STM32 Serial Protocol

Frame format:

```text
AA 55 + function_code(1) + data_len(1) + params(N) + crc(1)
```

Serial config:

```text
baudrate = 1000000
data bits = 8
stop bits = 1
parity = none
send format = HEX
byte order = little endian
```

CRC:

```text
CRC8-MAXIM / Dallas
poly = 0x31
refin = true
refout = true
init = 0x00
xorout = 0x00
calculation range = function_code + data_len + params
```

Verified usable frames:

```text
LED blink:
AA 55 01 07 01 E8 03 E8 03 0A 00 D9

Buzzer 1000Hz, on 1s, off 1s, repeat 3:
AA 55 02 08 E8 03 E8 03 E8 03 03 00 E4
```

Function codes:

```text
0x00 SYS
0x01 LED
0x02 BUZZER
0x03 MOTOR
0x04 PWM_SERVO
0x05 BUS_SERVO
0x06 KEY
0x07 IMU
0x08 GAMEPAD
0x09 SBUS
```

## Current Code

User-accessible copy:

```text
/home/sunrise/rdk_robot/
```

Workspace copy:

```text
/root/.openclaw/workspace/rdk_robot/
```

Files:

- `protocol.py`: STM32 protocol frame builders and CRC
- `control_mapping.py`: project-specific gamepad-to-command mapping
- `joystick_control.py`: reads gamepad and prints mapped command values only
- `robot_teleop.py`: reads gamepad and can send STM32 serial frames
- `test_protocol.py`: prints sample protocol frames and CRC checks

Run print-only joystick mapping:

```bash
cd /home/sunrise/rdk_robot
python3 joystick_control.py
```

Run teleop dry-run, no serial writes:

```bash
cd /home/sunrise/rdk_robot
python3 robot_teleop.py --dry-run
```

Run teleop with serial output:

```bash
cd /home/sunrise/rdk_robot
python3 robot_teleop.py --send
```

Use stable serial path if needed:

```bash
python3 robot_teleop.py --send --port /dev/serial/by-id/usb-1a86_USB_Single_Serial_5B32012490-if00
```

## Safety Notes

Before running `robot_teleop.py --send`:

- Lift the robot or disconnect motor power for first test
- Confirm motor IDs and directions
- Confirm PWM servo IDs and safe pulse ranges
- Confirm bus servo ID and safe pulse range
- Current servo IDs and pulse widths in `robot_teleop.py` are placeholders and must be tuned on hardware

---

## 视觉方案调试记录（2026-05-29）

### 全链路架构

```
Astra Mini S (USB)
  → astra_depth_streamer (C++, Astra SDK, 8090端口, HTTP MJPEG)
    → astra_frame_bridge.py (Python, cv2.VideoCapture, 帧文件桥接)
      → /tmp/rdk_robot_frames/latest_rgb.jpg + latest_depth16.pgm
        → bpu_perception_demo.py (BPU YOLOv5s + 深度测距 + 豆包VLM总结, 8091端口)
          → 浏览器网页 (MJPEG推流 + 信息面板)
```

### 踩坑 1：astra_depth_streamer 未运行

**现象**：8091 网页显示 HTML 占位，无实况画面。`/tmp/rdk_robot_frames/` 目录为空。

**原因**：`astra_depth_streamer` 是连接 Astra 相机和感知服务的关键桥梁，负责从 Astra SDK 拉取 RGB+深度帧并封装为 HTTP MJPEG 流，但它没有在运行。

**修复**：
```bash
cd /home/sunrise/rdk_robot
export ASTRA_SDK_ROOT=/home/sunrise/AstraSDK
export LD_LIBRARY_PATH="$ASTRA_SDK_ROOT/lib:$ASTRA_SDK_ROOT/lib/Plugins:${LD_LIBRARY_PATH:-}"
./astra_depth_streamer 8090 &
```

### 踩坑 2：streamer 与 perception demo 格式不匹配

**现象**：streamer 提供 HTTP MJPEG（8090 端口），但 `bpu_perception_demo.py` 从文件系统读取帧。

**原因**：中间缺一个桥接——streamer 输出 HTTP 流，perception 期望文件。

**修复**：编写 `/home/sunrise/rdk_robot/astra_frame_bridge.py`，用 `cv2.VideoCapture` 从 streamer 拉 RGB+深度 MJPEG 帧，对深度帧做 JET colormap 反向映射（HSV Hue → 16-bit mm），写入 `latest_rgb.jpg` + `latest_depth16.pgm`。

### 踩坑 3：文件写入竞态（JPEG 损坏）

**现象**：bridge 写文件时 perception 同时读，产生 `Corrupt JPEG data` 警告，检测结果不稳定。

**原因**：bridge 用 `write_bytes()` 直接覆盖，perception 的 `cv2.imread()` 可能在文件写到一半时读取。

**修复**：改为原子写入——先写 `.tmp` 临时文件，再 `os.rename()` 到目标路径（rename 在 Linux 上是原子操作）。

### 踩坑 4：MJPEG 缓冲区堆积导致高延时

**现象**：网页画面延时 1-2 秒。

**原因**：
- bridge 的 `LOOP_INTERVAL = 0.3` 秒，拉帧太慢
- `cv2.VideoCapture` 对 MJPEG 流内部缓冲多帧，每次 `read()` 拿到的是旧帧
- 尝试 `grab()` 多帧刷新缓冲→每个 `grab()` 实际解码 JPEG，4 次 grab 耗时 1000ms+
- bpu_perception loop 的 `time.sleep(0.15)` 增加了额外延时
- MJPEG 推送只在帧变化时推（`data != last`），跳帧

**修复**：
1. bridge 间隔从 0.3s → 0.005s（紧循环，靠处理时间自然限速）
2. 设置 `cv2.CAP_PROP_BUFFERSIZE = 1` 限制 OpenCV 内部缓冲
3. 去掉 grab 刷新（它反而解码帧，增延迟）
4. perception loop sleep 从 0.15s → 0.01s
5. MJPEG 推送去掉 `data != last` 跳帧逻辑，每帧都推
6. MJPEG 推送间隔从 0.15s → 0.05s

**结果**：总管道延时从 ~1-2 秒降到 ~200-300ms。

### 踩坑 5：检测目标数为 0

**现象**：web 画面有图像但无任何 YOLO 检测框，`result.json` 返回 `detections=[]`。前方 0.4m（STOP 区域）。

**排查过程**：

1. **阈值排查**：降到 0.01 仍然 0 检测 → 排除阈值问题
2. **分辨率排查**：streamer 源码 `DOWNSCALE = 4`，Astra 原始 640×480 → 输出 160×120，分辨率太低
3. **后处理库排查**：用 640×480 合成测试图（含绿色矩形+红色圆形），0 检测；报 `yolov5x unsupport shift dequantzie now!` → 怀疑 `libpostprocess.so` 与模型输出格式不兼容（模型输出 float32，后处理期望量化 int32）
4. **最终确认**：用完整的 `hbDNNTensor_t` 结构体（quantiType=0, 设置 validShape）重新测试 → 合成图成功检出 2 个目标（frisbee 0.51, sports ball 0.51）→ **后处理库和模型都没问题**
5. **确定性验证**：同一帧 ×30 次推理，100% 一致检出 boat 0.2438 → 模型推理完全确定
6. **跨帧采样**：20 帧实时采样，70% (14/20) 检出 → 帧间差异导致置信度在阈值边缘抖动

**最终根因 1 — 分辨率**：`DOWNSCALE = 4` 把 640×480 缩到 160×120，YOLOv5s 输入 672×672，超 4 倍上采样 → 物体只有几个像素，检测极度困难。

**最终根因 2 — JPEG 压缩质量**：`JPEG_QUALITY = 45`，320×240 下物体就 ~16×6 像素，压缩伪影淹没特征。置信度在阈值 0.2 附近抖动导致 30% 帧丢失。

**修复**：
1. `DOWNSCALE: 4 → 2`，输出 320×240，物体像素翻 4 倍
2. `JPEG_QUALITY: 45 → 85`，几乎无损，压缩伪影大幅减少
3. 重新编译 `astra_depth_streamer`（链接 `-lastra -lastra_core_api -lastra_core -ljpeg -lpthread`）
4. 置信度阈值从 0.45 → 0.2（临时）→ 0.35（稳定后）

**结果**：检测从 0 → 5 个目标（microwave 0.525, refrigerator 0.417, tv 0.312, oven 0.244, microwave 0.219），阈值过滤后保留 2 个高质量检测。

### 踩坑 6：页面布局

**现象**：画面占比太小，iframe 信息面板占太多空间；调整后又发现工作总结被截断。

**修复**：最终保留原始比例——img 72vh + panels 34vh，padding 6px。

### 踩坑 7：豆包工作总结刷新策略

**原逻辑**：固定 5 秒间隔发送。无论豆包是否返回，到 5 秒就堆新请求。

**问题**：豆包 VLM 调用可能需要数秒，5 秒固定间隔会导致：
- 豆包还在处理，新请求堆积
- 豆包已返回，白白等到 5 秒

**修复**：改为"返回即发"模式——
- 新增 `summary_min_gap = 2.0`（两次请求最小间隔，防止空转）
- 触发条件：`summary_busy == False AND 距上次请求 ≥ 2 秒`
- 豆包返回设置 `summary_busy = False` → 下一个感知帧检查通过 → 立即用最新数据发下一次
- `period_s` 改为实际经过时间而非固定值

### 豆包 VLM 工作总结链路

1. `record_work_history()` 每 1 秒收集一次感知快照（检测目标、距离、风险、动作），保留最近 60 条
2. 触发条件满足时，取最近 15 帧工作历史 + 当前标注画面 JPEG → 子线程调用豆包 `doubao-seed-2-0-mini-260428`
3. Prompt：机器人巡检员第一人称口头播报，120-180 字，自然口语化，说明环境、谨慎程度、下一步倾向
4. VLM 成功 → `status=vlm_ok`；失败 → `status=fallback`（纯规则统计兜底）
5. 面板 `/panels.html` 每 1 秒刷新展示最新总结

### 当前全链路参数

| 组件 | 参数 | 值 |
|------|------|-----|
| streamer | DOWNSCALE | 2 (320×240) |
| streamer | JPEG_QUALITY | 85 |
| streamer | CAPTURE_INTERVAL | 250ms |
| bridge | 文件写入 | 原子 rename |
| bridge | OpenCV buffer | CAP_PROP_BUFFERSIZE=1 |
| bridge | 循环间隔 | ~0.005s 紧循环 |
| perception | 模型 | yolov5s_672x672_nv12.bin |
| perception | 置信度阈值 | 0.35 |
| perception | loop sleep | 0.01s |
| perception | summary 最小间隔 | 2.0s |
| perception | summary 触发 | busy=False + 距上次≥2s |
| VLM | 模型 | doubao-seed-2-0-mini-260428 |
| VLM | 端点 | ark.cn-beijing.volces.com |

### 启动命令备忘

```bash
# 1. 启动 Astra streamer
cd /home/sunrise/rdk_robot
export ASTRA_SDK_ROOT=/home/sunrise/AstraSDK
export LD_LIBRARY_PATH="$ASTRA_SDK_ROOT/lib:$ASTRA_SDK_ROOT/lib/Plugins:${LD_LIBRARY_PATH:-}"
nohup ./astra_depth_streamer 8090 > /tmp/astra_streamer.log 2>&1 &

# 2. 启动帧桥接
nohup python3 astra_frame_bridge.py > /tmp/astra_bridge.log 2>&1 &

# 3. 启动 BPU 感知 + 豆包总结
nohup python3 bpu_perception_demo.py --port 8091 > /tmp/bpu_perception.log 2>&1 &

# 网页访问
http://<board-ip>:8091/
```

### 停止命令备忘

```bash
pkill -f "astra_depth_streamer"
pkill -f "astra_frame_bridge"
pkill -f "bpu_perception_demo"
```

---

## Next Work

- Confirm motor ID/order and direction
- Confirm mecanum wheel mixing signs
- Confirm PWM servo IDs and pulse limits
- Confirm bus servo ID and limits
- Add explicit STM32 command mode for camera lift, camera posture, and shape transitions
- Integrate Astra depth obstacle detection after manual teleop is safe
