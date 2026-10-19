#!/usr/bin/env bash
set -e
cd "$(dirname "$0")/.."
source scripts/env.sh
export PYTHONPATH="$WORKSPACE_ROOT/.venv/lib/python3.12/site-packages:${PYTHONPATH:-}"
exec python3 scripts/run_demo.py "$@"
