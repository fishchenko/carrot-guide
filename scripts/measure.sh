#!/usr/bin/env bash
# Every measurement the README quotes, in one pass, so the numbers there can be
# regenerated rather than trusted. Assumes the simulator is up (make sitl-up).
#
# Takes about 35 minutes: most of it is real flying time, plus the ten-minute
# profiling run at the end.
set -euo pipefail

cd "$(dirname "$0")/.."

BIN=${BIN:-.venv/bin}
URL=${SITL_URL:-tcp:127.0.0.1:5760}
OUT=${OUT:-docs/measurements}

# The lookahead that cancels the vehicle's turn lag on the orbit: set from the
# measured command-to-reaction latency, not guessed.
LOOKAHEAD=${LOOKAHEAD:-0.22}

mkdir -p "$OUT" logs docs

# Summaries are written by the CLI itself (--summary) rather than captured from stdout:
# under memray the profiler prints its own banner around the JSON, which left two of
# these files unparseable for as long as they had been committed.
run() { echo "== $1" >&2; shift; "$BIN/python" -m carrot_guide.cli "$@" >/dev/null; }

# memray's own report carries absolute paths from whatever checkout produced it. These
# files are committed, so the home directory comes out before they land on disk.
stats() { "$BIN/memray" stats "$1" | sed "s|$HOME|~|g" >"$2"; }

run "hold, 60 s, still air" hold --url "$URL" \
    --north 25 --altitude 15 --seconds 60 \
    --log logs/hold-calm.csv --summary "$OUT/hold-calm.json"

run "hold, 60 s, steady 6 m/s wind" hold --url "$URL" \
    --north 25 --altitude 15 --wind 6 --seconds 60 \
    --log logs/hold.csv --summary "$OUT/hold.json"

run "hold, 60 s, gusty 6 m/s wind" hold --url "$URL" \
    --north 25 --altitude 15 --wind 6 --turbulence 1 --seconds 60 \
    --log logs/hold-gusty.csv --summary "$OUT/hold-gusty.json"

run "hold, 60 s, 15 m/s wind with heavy gusts" hold --url "$URL" \
    --north 25 --altitude 15 --wind 15 --turbulence 5 --seconds 60 \
    --log logs/hold-storm.csv --summary "$OUT/hold-storm.json"

run "orbit, 90 s, 25 m, gusty wind, tangent aimed at the present position" orbit --url "$URL" \
    --radius 25 --speed 4 --altitude 20 --wind 6 --turbulence 1 --seconds 90 \
    --log logs/orbit-nolookahead.csv --summary "$OUT/orbit-nolookahead.json"

run "orbit, 90 s, 25 m, gusty wind, ${LOOKAHEAD} s lookahead" orbit --url "$URL" \
    --radius 25 --speed 4 --altitude 20 --wind 6 --turbulence 1 --seconds 90 \
    --lookahead "$LOOKAHEAD" --log logs/orbit.csv --summary "$OUT/orbit.json"

run "command latency, 6 trials" latency --url "$URL" --trials 6 \
    --summary "$OUT/latency.json"

echo "== plots" >&2
BIN="$BIN" scripts/plots.sh

# A leak would give itself away as a peak that grows with the length of the run, so
# each mode is profiled at two durations an order of magnitude apart. Both are flown in
# the same gusty wind as the runs above, so the memory numbers and the tracking numbers
# describe the same experiment.
for seconds in 60 600; do
    echo "== ${seconds} s under memray, keeping every cycle" >&2
    "$BIN/memray" run --force -o "memray-hold-${seconds}.bin" -m carrot_guide.cli hold \
        --url "$URL" --north 25 --altitude 15 --wind 6 --turbulence 1 --seconds "$seconds" \
        --log "logs/hold-${seconds}s.csv" --summary "$OUT/hold-${seconds}s.json" >/dev/null
    stats "memray-hold-${seconds}.bin" "$OUT/memray-${seconds}s.txt"

    echo "== ${seconds} s under memray, streaming to disk only" >&2
    "$BIN/memray" run --force -o "memray-stream-${seconds}.bin" -m carrot_guide.cli hold \
        --url "$URL" --north 25 --altitude 15 --wind 6 --turbulence 1 --seconds "$seconds" \
        --stream-only --log "logs/hold-stream-${seconds}s.csv" \
        --summary "$OUT/hold-stream-${seconds}s.json" >/dev/null
    stats "memray-stream-${seconds}.bin" "$OUT/memray-stream-${seconds}s.txt"
done

echo "done; results in $OUT"
