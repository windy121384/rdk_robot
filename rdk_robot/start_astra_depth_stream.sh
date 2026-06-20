#!/bin/sh
set -eu
cd /home/sunrise/rdk_robot
export ASTRA_SDK_ROOT=/home/sunrise/AstraSDK
export LD_LIBRARY_PATH="$ASTRA_SDK_ROOT/lib:$ASTRA_SDK_ROOT/lib/Plugins:${LD_LIBRARY_PATH:-}"
PORT="${1:-8090}"
exec ./astra_depth_streamer "$PORT"
