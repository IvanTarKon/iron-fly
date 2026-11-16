#!/usr/bin/env bash
# Default is strict: known upstream failures remain a nonzero overall status.
set -eu
cd "$(dirname "$0")/.."
mode=${1:---strict}
case "$mode" in --strict|--demo-only) ;; *) echo 'Usage: check.sh [--strict|--demo-only]' >&2; exit 2;; esac
PYTHON=${PYTHON:-.venv/bin/python}
mkdir -p build reports
"$PYTHON" scripts/verify_sources.py > reports/sources.txt 2>&1
bash scripts/build.sh > reports/bridge-build.txt 2>&1
fw=vendor/Hexapod-Simulation/Hexapod-Firmware
upstream=0
bash "$fw/test/run.sh" > reports/pinned-upstream-tests.txt 2>&1 || upstream=$?
export UPSTREAM_EXIT=$upstream
own=0
run_check() {
  name=$1
  shift
  code=0
  "$@" > "reports/$name.txt" 2>&1 || code=$?
  echo "$name exit=$code"
  if [ "$code" -ne 0 ]; then own=1; cat "reports/$name.txt"; fi
}
run_check audit "$PYTHON" scripts/audit_upstream.py
run_check motion-tests "$PYTHON" tests/test_motion.py
run_check brain-tests "$PYTHON" tests/test_brain_control.py
run_check trajectory-build "${CXX:-g++}" -std=c++17 -Wall -Wextra -Werror -O2 -I"$fw/src" tests/trajectory_probe.cpp -o build/trajectory_probe
if [ "$own" -eq 0 ]; then run_check trajectory ./build/trajectory_probe; fi
run_check simulation-run "$PYTHON" scripts/sim_probe.py
export DEMO_EXIT=$own CHECK_MODE=$mode
"$PYTHON" - <<'PY'
import json, os
from pathlib import Path
report = {k: os.environ[k] for k in ('UPSTREAM_EXIT','DEMO_EXIT','CHECK_MODE')}
Path('reports/check-summary.json').write_text(json.dumps(report, indent=2))
print(report)
PY
if [ "$own" -ne 0 ]; then exit "$own"; fi
if [ "$mode" = --strict ]; then exit "$upstream"; fi
