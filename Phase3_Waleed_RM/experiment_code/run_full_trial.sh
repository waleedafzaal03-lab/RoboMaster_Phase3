#!/usr/bin/env bash
set -euo pipefail

trial="${1:-}"
case "$trial" in
  0[1-9]|10) ;;
  *) echo "Usage: bash run_trial.sh 01 ... 10"; exit 2 ;;
esac

root="$HOME/robomaster_phase3/lecture11_wheel_error"
logs="$root/logs"
stamp="$(date +%Y%m%d_%H%M%S)"
rec_log="$logs/full${trial}_${stamp}_recorder.txt"
pilot_log="$logs/full${trial}_${stamp}_pilot.txt"
export PYTHONPATH="$HOME/robomaster_phase3/code/rm_phase3_estimation:${PYTHONPATH:-}"

echo "Starting trial $trial recorder..."
python3 "$root/code/record_wheel_camera.py" \
  --label "full${trial}" --duration 34 >"$rec_log" 2>&1 &
rec_pid=$!
trap 'kill -INT "$rec_pid" 2>/dev/null || true' EXIT

sleep 1
echo "Starting pilot; watch the robot. Ctrl+C stops the pilot."
set +e
timeout --signal=INT --kill-after=3s 27s \
  python3 "$root/code/pilot_20cm.py" \
  --ros-args -p enable_motion:=true -p pilot_stop_tof_m:=0.23 \
  2>&1 | tee "$pilot_log"
pilot_status=${PIPESTATUS[0]}
set -e

wait "$rec_pid" || { cat "$rec_log"; exit 1; }
trap - EXIT
cat "$rec_log"
echo "Pilot log: $pilot_log"

if [[ "$pilot_status" != 0 && "$pilot_status" != 124 ]] ||
   ! grep -Eq 'PILOT_STOP_CAMERA|PILOT_STOP_TOF' "$pilot_log"; then
  echo "Trial did not finish normally; review it before counting it."
  exit 1
fi
echo "Full trial $trial recording complete."
