#!/usr/bin/env bash
# Laptop demo (Must item 2): host tests, pytest, then the closed-loop sim landing.
set -e
cd "$(dirname "$0")/.."
PY=.venv/bin/python
make -s -C gimbal_fw/host_test
$PY -m pytest -q
$PY -m landing_sw.main --sim
