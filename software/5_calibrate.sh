#!/usr/bin/env bash
# Guided resistor calibration (board must run tools/board_check)
cd "$(dirname "$0")"
.venv/bin/python calibrate.py "$@"
