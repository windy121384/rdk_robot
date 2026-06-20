#!/bin/bash
# 手动模式启动 — 手柄优先
set -e
cd /home/sunrise/rdk_robot
pkill -f manual_mode 2>/dev/null; sleep 0.5
nohup python3 -u manual_mode.py > /tmp/manual_mode.log 2>&1 &
sleep 2
echo "=== 手动模式 v2 ==="
pgrep -af manual_mode
head -8 /tmp/manual_mode.log