#!/usr/bin/env bash
# Analyse a session: the given folder, or the most recent one in sessions/
cd "$(dirname "$0")"
DIR="${1:-$(ls -d sessions/*/ 2>/dev/null | sort | tail -n 1)}"
if [ -z "$DIR" ]; then
    echo "No sessions found in sessions/"
    exit 1
fi
.venv/bin/python analyze.py "$DIR"
xdg-open "$DIR/analysis" >/dev/null 2>&1 &
