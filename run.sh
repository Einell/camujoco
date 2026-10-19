#!/usr/bin/env bash
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
python_bin=python3
if [[ -x "$project_dir/.venv/bin/python" ]]; then
  python_bin="$project_dir/.venv/bin/python"
fi
export PYTHONPATH="$project_dir/src${PYTHONPATH:+:$PYTHONPATH}"
if [[ -d "$project_dir/.runtime/python-deps/PySide6" ]]; then
  export PYTHONPATH="$project_dir/.runtime/python-deps:$PYTHONPATH"
fi
local_libs="$project_dir/.runtime/system-libs/usr/lib/x86_64-linux-gnu"
if [[ -d "$local_libs" ]]; then
  export LD_LIBRARY_PATH="$local_libs${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi
export MUJOCO_GL="${MUJOCO_GL:-egl}"
exec "$python_bin" -m camera_lab.app "$@"
