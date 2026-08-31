#!/usr/bin/env bash
# Every figure the README embeds, from the log that produced it.
#
# One definition, called by both `make plots` and scripts/measure.sh. They used to hold
# separate lists that had drifted apart: docs/hold.png came from a different run
# depending on which one you ran last, and docs/orbit-nolookahead.png was missing from
# `make plots` altogether, so regenerating the figures quietly left one of them stale.
set -euo pipefail

cd "$(dirname "$0")/.."

. ./.env
BIN=$CARROT_GUIDE_VENV/bin

plot() { "$BIN/python" -m carrot_guide.cli report "$1" --circle "$2" --plot "$3" >/dev/null; }

# Same, for the runs whose target moves: `--target` draws the line it ran along rather
# than a cross it never sat on.
chase() { "$BIN/python" -m carrot_guide.cli report "$1" --target "$2" --plot "$3" >/dev/null; }

# The gusty run is the one the README's hold section discusses, so it is the one drawn.
plot logs/hold-gusty.csv        "25,0"    docs/hold.png
plot logs/orbit.csv             "0,0,25"  docs/orbit.png
plot logs/orbit-nolookahead.csv "0,0,25"  docs/orbit-nolookahead.png

# The target of the intercept pair in measure.sh: 60 north, 40 west, running due east
# at 3 m/s. Both laws are flown at it, so both figures carry the same dashed line.
chase logs/intercept-pronav.csv  "60,-40,0,3" docs/intercept-pronav.png
chase logs/intercept-pursuit.csv "60,-40,0,3" docs/intercept-pursuit.png
