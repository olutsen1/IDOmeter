#!/usr/bin/env bash
# The app with the device simulator (no hardware needed)
cd "$(dirname "$0")"
.venv/bin/python simulator.py &
SIM=$!
trap 'kill $SIM 2>/dev/null' EXIT
sleep 1
.venv/bin/python receiver.py --sim
