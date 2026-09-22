#!/usr/bin/env bash
set -euo pipefail
calibration_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
calibration_python="$calibration_dir/.venv/bin/python"
if [[ ! -x "$calibration_python" ]]; then
    echo "未找到工具目录的 .venv，请先按 README.md 安装依赖。" >&2
    exit 1
fi
exec "$calibration_python" -m streamlit run "$calibration_dir/app.py" \
    --server.address=127.0.0.1 --server.headless=true --browser.gatherUsageStats=false "$@"
