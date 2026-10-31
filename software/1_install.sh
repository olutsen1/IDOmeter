#!/usr/bin/env bash
# One-time setup on Linux (Ubuntu/Debian): a private Python environment in .venv
set -e
cd "$(dirname "$0")"

if ! python3 -c "import venv, ensurepip" 2>/dev/null; then
    echo "Python's venv module is missing; installing it (asks for your password)..."
    sudo apt install -y python3-venv python3-pip
fi
# Qt needs this library to open windows on Ubuntu; it is often not installed
if ! ldconfig -p | grep -q "libxcb-cursor.so.0"; then
    echo "Installing libxcb-cursor0, needed by Qt (asks for your password)..."
    sudo apt install -y libxcb-cursor0
fi

python3 -m venv .venv
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install -r requirements.txt
echo
echo "Done. Start the demo with ./2_demo_with_simulator.sh"
