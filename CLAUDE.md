# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

Own guidance laws (point hold, circular orbit) for a multirotor, flown against ArduPilot SITL over
MAVLink at a 10 Hz control loop. Simulator only — no hardware. `SPEC.md` is the original brief;
`README.md` carries every measured number and the reasoning behind it.

## Commands

```bash
cp .env.example .env          # required: set CARROT_GUIDE_VENV to an absolute path
make venv                     # virtualenv + editable install with [plots,dev]
make test                     # 127 unit tests, no simulator, ~1.6 s
make smoke                    # the installed console script actually imports
make sitl-up                  # build & run ArduCopter SITL in Docker (first build ~15 min)
make test-sitl                # 5 integration tests against a running SITL (~3 min cold)
make check                    # test, smoke, sitl-up, wait for port 5760, test-sitl
make hold / make orbit        # the two flight experiments
make latency                  # command-to-reaction latency
make profile                  # 10-minute run under memray
make measure                  # everything the README quotes, one pass (~35 min)
make plots                    # regenerate docs/*.png from logs/*.csv
make sitl-down                # stop the simulator
```

`make test` passes with a broken install, because `pythonpath = ["src"]` in
`pyproject.toml` lets pytest import from the source tree — that is what `make smoke`
exists to catch.

Single test: `$CARROT_GUIDE_VENV/bin/pytest tests/test_guidance.py::test_name -q`. SITL tests
are marked `sitl` and skip unless `CARROT_SITL_URL` is set (`make test-sitl` sets it). `pytest`
needs no install step — `pythonpath = ["src"]` is in `pyproject.toml`.

Flying commands go through the `carrot-guide` CLI (`carrot_guide.cli`): `telemetry`, `hold`,
`orbit`, `latency`, `report`. Each flying subcommand writes a CSV to `logs/` and prints a JSON
summary on stdout; `report <log>` re-derives the same numbers (and plots) from a log afterwards.

## Architecture

The one boundary that matters is I/O. `link.py` is the **only** module that touches a socket;
everything above it works on plain values, so all the math and the whole control loop are tested
without a simulator.

- `state.py` — `NED` vector, `GlobalPosition`, flat-earth WGS84 ↔ NED projection
- `guidance.py` — `HoldPoint` (PD outer loop) and `Orbit` (tangential + radial terms). Pure math,
  no imports from `link`/`telemetry`. Both return a `VelocityCommand` in local NED; the autopilot
  closes the inner velocity loop.
- `telemetry.py` — `TelemetryTracker`: last-known-value view fed one MAVLink message at a time.
  No pymavlink import, so parsing is tested against hand-built stub messages.
- `link.py` — `MavlinkLink`: commands, ACK/state-change confirmation, params, `send_velocity`.
- `runner.py` — `FixedRateLoop` + `GuidanceRunner`, and the `GuidanceLaw`/`VehicleLink` Protocols
  that define what the loop needs from either side.
- `mission.py` — `launch()`, `airborne()` context manager, latency measurement.
- `utils.py` — everything general, types and functions alike: `Deadline` (the monotonic countdown
  every wait in `link`/`mission`/`cli` is built from), `Command` (the argparse subcommand base),
  `percentile()` shared by the loop and the analysis so neither imports the other, the CSV text
  parsers, and `emit_json()` with its flag.
- `metrics.py`, `recording.py`, `plots.py`, `cli.py` — summaries, CSV log, figures, subcommands.

### Invariants worth not breaking

- **Clock and sleep are injected.** `FixedRateLoop(monotonic=…, sleep=…)` and
  `GuidanceRunner(loop_factory=…)` exist so ten minutes of loop scheduling is a millisecond-long
  test with a fake clock. Keep new timing code parameterised the same way.
- **Deadlines are `origin + index * period`**, never accumulated by addition — float drift lost
  ticks on long runs, and a test asserts every tick lands on its multiple.
- **Spin slack is measured, not assumed.** `FixedRateLoop.calibrated()` calls
  `measure_sleep_overshoot()` with a request close to the real period, because Darwin's sleep
  overshoot grows with the requested interval. `sleep(0)` in the spin is worse than a real spin.
- **Stale telemetry stops the loop.** No position report for `stale_after_s` → send zero velocity
  and raise `StaleTelemetry` rather than fly on a dead reckoning.
- **Runs end with a zero-velocity command.** ArduPilot treats velocity targets as short-lived and
  brakes when they stop arriving; that is the intended failsafe.
- **`launch()` is a retry loop, not a sequence.** A cold autopilot drops GUIDED, refuses pre-arm,
  and self-disarms after ~10 s; each step is re-established immediately before it is needed, and
  the failure reason comes from `STATUSTEXT`, not `COMMAND_ACK`.
- **Metrics need the whole series** (settle time is found by scanning backwards), so summaries must
  not be computed inside a long-lived flying process — that is what `--stream-only` plus
  `carrot-guide report` is for. Adding a summary to the streaming path reintroduces the memory
  growth it exists to avoid.
- **Reaching the target and holding it are separate events.** `settle_time()` is when the error
  last crosses the threshold; the error statistics start `DEFAULT_HOLD_LEAD_IN_S` later, because at
  the crossing the outer loop is still an exponential from steady state and the tail would dominate
  the average (it did: it made a 60 s run look four times worse than a 600 s one). Every summary
  names the `window` it used — `hold`, `post-settle` or `whole run` — so a run that never settled
  cannot be mistaken for one that did.
- **The socket has one reader.** `recv_match(type=…)` consumes and discards non-matching messages,
  so a typed read eats the `STATUSTEXT` that explains a refusal. All waits go through
  `MavlinkLink._recv_matching`, which feeds the tracker and filters afterwards.
- **Lateness cannot see a stall.** A resync re-bases the schedule, including the deadline the next
  tick measures itself against, so any overrun past ~2 periods reports as zero lateness. Quote
  `resyncs`/`skipped_cycles` alongside it or the timing numbers mean nothing.
- **Wind is always applied**, including at zero. Skipping the parameter writes for a calm run left
  it flying in whatever the previous run had set.
- **`Orbit.lookahead_s` is a measured value**, equal to the command-to-reaction latency of this
  link (0.22 s here), not a tuned gain. If the vehicle or link changes, re-measure with
  `make latency` rather than fitting it.
- The local frame is anchored at the takeoff point but at **home altitude**, matching
  `MAV_FRAME_LOCAL_NED`, so a target's `down` is minus the altitude above home.

## Conventions

- Prose for humans (`README.md`, `SPEC.md`) is **Ukrainian**; all code, comments, docstrings, test
  names and commit messages are **English**. Keep that split.
- Comments explain *why* — usually a measured fact or a failure that motivated the code. They are
  load-bearing documentation here; do not strip them, and match the register when adding code.
- Tests are named as sentences (`test_a_dead_position_stream_stops_the_loop_rather_than_flying_blind`).
- No linter or formatter is configured; match surrounding style (~100 col, `from __future__ import
  annotations`, frozen dataclasses for values).
- Numbers in the README come from `docs/measurements/*.json`, produced by `scripts/measure.sh`.
  If a change moves the numbers, regenerate rather than editing the tables by hand. Summaries are
  written with `--summary`, not captured from stdout — under `memray` the profiler's banner lands
  on stdout around the JSON and silently corrupted two committed files that way.
- `scripts/plots.sh` is the single definition of the figures; `make plots` and `scripts/measure.sh`
  both call it, because two lists of plot commands had already drifted apart.
- **`utils.py` holds everything general**, types and functions alike, and admits only what would
  read the same way *verbatim* in a project with no aircraft in it. Liftable is necessary but
  not sufficient: `FixedRateLoop` would lift cleanly and stays in `runner` anyway, because a
  10 Hz control loop is what this project *is*. `utils` takes what is incidental to the work —
  a countdown, a percentile, an argparse base — never the work itself. Anything that fails the
  test keeps its domain half at home: `Command` moved, the `--url` option group it used to carry
  stayed behind on `cli.VehicleCommand`.
- A subcommand is a `utils.Command` subclass in `cli.py` carrying its own options and its own
  handler, listed in `COMMANDS`. Each rung of the ladder owns the options at its level and
  nothing else: `Command` is generic argparse plumbing and lives in `utils`; `VehicleCommand`
  adds `--url`/`--timeout` for the four that talk to a vehicle (`report` does not); the flying
  two subclass `FlightCommand`. Kept apart, `--kd` drifted onto the orbit parser and was read
  by nothing. Shared option groups belong on the rung that owns them, never as a loose
  module-level function a subclass has to remember to call.

## Simulator

`docker-compose.yml` builds `docker/Dockerfile.sitl` from ArduPilot sources at the pinned tag
`Copter-4.5.7` (no usable prebuilt image exists; community ones are stale and amd64-only). Home is
Kyiv, MAVLink over TCP on `5760`. `SITL_SPEEDUP` must stay `1` for any timing measurement — the
guidance loop runs on the real clock, so speedup makes the measured rate meaningless. Wind is set
through parameters (`SIM_WIND_SPD`, `SIM_WIND_DIR`, `SIM_WIND_TURB`) from the run itself; steady
wind alone barely moves the error, turbulence is what makes a run worth quoting.
