#!/usr/bin/env python3
"""
astra_frame_bridge.py — Bridge between Astra depth streamer (port 8090)
and bpu_perception_demo.py.

Pulls RGB + depth MJPEG frames from the local streamer and writes them as:
  /tmp/rdk_robot_frames/latest_rgb.jpg       (JPEG, 3-ch BGR frame)
  /tmp/rdk_robot_frames/latest_depth16.pgm   (P5 PGM, 16-bit depth in mm)

Depth reverse-mapping: JET colormap → raw mm via HSV hue channel.
  Red (H≈0)  → near  ~400mm
  Blue (H≈120) → far  ~4000mm
"""

import os
import sys
import struct
import time
import signal
from pathlib import Path

import cv2
import numpy as np

# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------
RGB_URL = "http://127.0.0.1:8090/rgb.mjpg"
DEPTH_URL = "http://127.0.0.1:8090/depth.mjpg"
OUT_DIR = Path("/tmp/rdk_robot_frames")
RGB_PATH = OUT_DIR / "latest_rgb.jpg"
DEPTH_PATH = OUT_DIR / "latest_depth16.pgm"

LOOP_INTERVAL = 0.3          # seconds between frame pulls
DEPTH_NEAR_MM = 400          # mm for red (H≈0)
DEPTH_FAR_MM = 4000          # mm for blue (H≈120)
BUFFER_FLUSH_FRAMES = 4     # grab & discard to skip stale MJPEG buffer

SAT_THRESH = 40              # min saturation for valid depth pixel
VAL_THRESH = 20              # min value (brightness) for valid depth pixel

running = True


def on_shutdown(signum, frame):
    global running
    print("[bridge] Shutdown signal received, exiting loop.")
    running = False


signal.signal(signal.SIGINT, on_shutdown)
signal.signal(signal.SIGTERM, on_shutdown)


# ---------------------------------------------------------------------------
# JET colormap reverse mapping via HSV hue
# ---------------------------------------------------------------------------
def jet_hue_to_depth_mm(h: np.ndarray, s: np.ndarray, v: np.ndarray) -> np.ndarray:
    """
    Convert a JET-colormapped HSV image to raw depth in mm.

    Mapping (linear in hue):
      H=0   (red)  → 400 mm   (near)
      H=120 (blue) → 4000 mm  (far)

    Pixels with low saturation or value are treated as invalid → 0.
    """
    h = h.astype(np.float32)
    s = s.astype(np.float32)
    v = v.astype(np.float32)

    # Wrap-around red region: H∈(120, 180] maps back to [0, 60) red
    h = np.where(h > 120, 180.0 - h, h)

    # Clamp to valid hue range for JET (0 = red, 120 = blue)
    h = np.clip(h, 0.0, 120.0)

    # Linear depth mapping
    depth = DEPTH_NEAR_MM + (h / 120.0) * (DEPTH_FAR_MM - DEPTH_NEAR_MM)
    depth = depth.astype(np.uint16)

    # Mask out invalid pixels (too dark or desaturated)
    invalid = (s < SAT_THRESH) | (v < VAL_THRESH)
    depth[invalid] = 0

    return depth


# ---------------------------------------------------------------------------
# PGM P5 writer (16-bit, maxval=65535)
# ---------------------------------------------------------------------------
def write_pgm16(path: Path, depth: np.ndarray):
    """
    Write a uint16 numpy array as PGM P5.
    Uses little-endian byte order to match the np.frombuffer(dtype=uint16)
    reader in bpu_perception_demo.py on ARM64.
    """
    height, width = depth.shape
    header = f"P5\n{width} {height}\n65535\n".encode("ascii")
    # uint16 in native byte order (little-endian on ARM64)
    raw = depth.tobytes()
    path.write_bytes(header + raw)


# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------
def main():
    global running

    OUT_DIR.mkdir(parents=True, exist_ok=True)

    # Open both MJPEG streams
    cap_rgb = cv2.VideoCapture(RGB_URL)
    cap_depth = cv2.VideoCapture(DEPTH_URL)

    # Reduce OpenCV internal MJPEG buffer to 1 frame
    cap_rgb.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    cap_depth.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    if not cap_rgb.isOpened():
        print("[bridge] ERROR: cannot open RGB stream", RGB_URL, file=sys.stderr)
        sys.exit(1)
    if not cap_depth.isOpened():
        print("[bridge] ERROR: cannot open depth stream", DEPTH_URL, file=sys.stderr)
        sys.exit(1)

    print(f"[bridge] Started. RGB={RGB_URL}  Depth={DEPTH_URL}")
    print(f"[bridge] Output: {RGB_PATH} / {DEPTH_PATH}")
    print(f"[bridge] Interval={LOOP_INTERVAL}s, depth range={DEPTH_NEAR_MM}-{DEPTH_FAR_MM}mm")

    frame_n = 0
    skip_report = 0

    while running:
        t0 = time.time()

        # --- RGB ---
        ret_rgb, rgb_frame = cap_rgb.read()
        if not ret_rgb:
            print("[bridge] WARN: RGB read failed", file=sys.stderr)
            time.sleep(0.1)
            continue

        ok, jpg = cv2.imencode(".jpg", rgb_frame, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
        if ok:
            tmp = str(RGB_PATH) + ".tmp"
            Path(tmp).write_bytes(jpg.tobytes())
            Path(tmp).rename(RGB_PATH)

        # --- Depth ---
        ret_d, depth_frame = cap_depth.read()
        if not ret_d:
            print("[bridge] WARN: depth read failed", file=sys.stderr)
            time.sleep(0.1)
            continue

        hsv = cv2.cvtColor(depth_frame, cv2.COLOR_BGR2HSV)
        h, s, v = hsv[:, :, 0], hsv[:, :, 1], hsv[:, :, 2]
        depth_mm = jet_hue_to_depth_mm(h, s, v)

        tmp_d = str(DEPTH_PATH) + ".tmp"
        write_pgm16(Path(tmp_d), depth_mm)
        Path(tmp_d).rename(DEPTH_PATH)

        # --- Report ---
        frame_n += 1
        elapsed = time.time() - t0

        if skip_report == 0:
            valid_px = (depth_mm > 0).sum()
            total_px = depth_mm.size
            d_min = depth_mm[depth_mm > 0].min() if valid_px > 0 else 0
            d_max = depth_mm.max()
            print(f"[bridge] #{frame_n:04d}  rgb={rgb_frame.shape}  "
                  f"depth_valid={valid_px}/{total_px}  d_range={d_min}-{d_max}mm  "
                  f"elapsed={elapsed*1000:.1f}ms", flush=True)
            skip_report = 10
        else:
            skip_report -= 1

        # Sleep to maintain target interval
        sleep_s = max(0.0, 0.005 - elapsed)
        if sleep_s > 0:
            time.sleep(sleep_s)

    # Cleanup
    cap_rgb.release()
    cap_depth.release()
    print("[bridge] Exited.", flush=True)


if __name__ == "__main__":
    main()
