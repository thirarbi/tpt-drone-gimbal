#!/usr/bin/env bash
# idf.py for gimbal_fw with the Python env the build was configured with.
#   scripts/fw.sh build
#   scripts/fw.sh -p /dev/ttyUSB0 flash monitor      (exit monitor: Ctrl+])
set -e
export IDF_PYTHON_ENV_PATH="$HOME/.espressif/tools/python/v6.1/venv"
. "$HOME/.espressif/v6.1/esp-idf/export.sh" > /dev/null
cd "$(dirname "$0")/../gimbal_fw"
idf.py "$@"
