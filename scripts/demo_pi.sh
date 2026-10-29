#!/usr/bin/env bash
# Pi demo (Should item): camera -> marker -> body frame, gimbal held at NADIR by the ESP32.
#   scripts/demo_pi.sh imx477            or   scripts/demo_pi.sh ov9281 --no-gimbal
set -e
cd "$(dirname "$0")/.."
CAM="${1:-ov9281}"; shift || true
python3 -m landing_sw.demo_vision --camera "$CAM" "$@"
