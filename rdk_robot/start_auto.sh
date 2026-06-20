#!/bin/bash
# 自动模式全链路启动 — 不依赖手柄
set -e
cd /home/sunrise/rdk_robot
echo "=== 1/4 Astra 深度相机 ==="
pkill -f astra_depth_streamer 2>/dev/null; sleep 0.3
nohup ./astra_depth_streamer > /tmp/astra_depth.log 2>&1 &
sleep 3
echo "=== 2/4 帧桥接 ==="
pkill -f astra_frame_bridge 2>/dev/null; sleep 0.3
nohup python3 -u astra_frame_bridge.py > /tmp/astra_bridge.log 2>&1 &
sleep 3
echo "=== 3/4 BPU 感知 ==="
pkill -f bpu_perception_demo 2>/dev/null; sleep 0.3
nohup python3 -u bpu_perception_demo.py > /tmp/bpu_perception.log 2>&1 &
sleep 5
echo "=== 4/4 规则控制器 ==="
pkill -f rule_controller 2>/dev/null; sleep 0.3
nohup python3 -u rule_controller.py > /tmp/rule_controller.log 2>&1 &
sleep 3
echo "=== 全链路在线 ==="
pgrep -af 'astra_depth|bridge|bpu_perception|rule_controller'
tail -3 /tmp/rule_controller.log | tr '\0' ' '