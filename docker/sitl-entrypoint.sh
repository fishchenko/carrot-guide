#!/bin/sh
# Start one ArduCopter SITL instance and keep it in the foreground.
#
# SPEEDUP lets the simulator run faster than wall clock. The guidance loop is timed
# against the real clock, so anything above 1 makes the measured rate meaningless —
# it is here for quick smoke runs only, and the measurement runs leave it at 1.
set -eu

HOME_POSITION="${SITL_HOME:-50.4501,30.5234,120,0}"
SPEEDUP="${SITL_SPEEDUP:-1}"
MODEL="${SITL_MODEL:-quad}"

echo "SITL: model=${MODEL} home=${HOME_POSITION} speedup=${SPEEDUP}"

exec arducopter \
    --model "${MODEL}" \
    --home "${HOME_POSITION}" \
    --speedup "${SPEEDUP}" \
    --defaults /sitl/copter.parm \
    --serial0 tcp:0 \
    --instance 0 \
    --sim-address 0.0.0.0 \
    "$@"
