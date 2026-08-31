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
make test                     # 158 unit + 12 component, no simulator, ~0.7 s
make smoke                    # the installed console script actually runs
make sitl-up                  # build & run ArduCopter SITL in Docker (first build ~15 min)
make test-sitl                # 6 integration tests against a running SITL (~3 min cold)
make check                    # test, smoke, sitl-up, wait for port 5760, test-sitl
make hold / orbit / intercept # the three flight experiments
make latency                  # command-to-reaction latency
make profile                  # 10-minute run under memray
make measure                  # everything the README quotes, one pass (~38 min)
make plots                    # regenerate docs/*.png from logs/*.csv
make sitl-down                # stop the simulator
```

`make test` passes with a broken install, because `pythonpath = ["src"]` in
`pyproject.toml` lets pytest import from the source tree — that is what the smoke tests
exist to catch.

Single test:
`$CARROT_GUIDE_VENV/bin/pytest tests/carrot_guide/guidance/unit/test_hold.py::test_name -q`.
A whole kind is selected by marker — `-m unit`, `-m component`, `-m smoke`,
`-m integration` — which is what the three targets above do (`make test` takes the first
two). The integration tests skip unless `CARROT_SITL_URL` is set (`make test-sitl` sets it).
`pytest` needs no install step for the fast tests —
`pythonpath = ["src"]` is in `pyproject.toml`.

Flying commands go through the `carrot-guide` CLI (`carrot_guide.cli`): `telemetry`, `hold`,
`orbit`, `latency`, `report`. Each flying subcommand writes a CSV to `logs/` and prints a JSON
summary on stdout; `report <log>` re-derives the same numbers (and plots) from a log afterwards.

## Architecture

The one boundary that matters is I/O. `link.py` is the **only** module that touches a socket;
everything above it works on plain values, so all the math and the whole control loop are tested
without a simulator.

One class per file **where the class earns a file**. Value records stay grouped (`values.py`
holds `Limits`, `Gains` and `VelocityCommand` — 16 lines of code between them), an exception stays
beside what raises it, and the two Protocols stay together because they are one statement about
the loop's two sides. Six files hold more than one class for those reasons; a rule that produced
a 4-line file per 2-line dataclass would cost more than it buys.

Seven of the twelve modules became packages **keeping their own name**, so
`from carrot_guide.<name> import <symbol>` still resolves for those seven. `commands/` is the
exception and the one path that did change: it came out of `cli.py`, so the command classes moved
from `carrot_guide.cli` to `carrot_guide.commands.<name>`. `cli.py` itself keeps only `COMMANDS`,
`build_parser` and `main`.

Three rules keep the graph the acyclic DAG it already was: a package `__init__.py` re-exports only
its own submodules and never across a layer; submodules import siblings by full dotted path
(`from carrot_guide.guidance.values import Gains`), never through the package root; and **no
re-exported name may equal a submodule name** — `from .launch import launch` overwrites the
submodule attribute, so `mission.launch` became the function and `mission/launch.py` was
unreachable by path. That is why the files are `mission/bringup.py` and `utils/percentiles.py`.

- `state.py` — `NED` vector, `GlobalPosition`, `VehicleState`, flat-earth WGS84 ↔ NED projection.
  Still flat: three value types and the projection pair, which belongs to two of them.
- `guidance/` — pure math, no imports from `link`/`telemetry`. `values` (`Limits`, `Gains`,
  `VelocityCommand`), `vectors` (`_rotated`, `_horizontal_unit`, `bearing_deg`, `YAW_DEADBAND_M` —
  everything with more than one caller; the private ones stay private, a sibling module imports
  them by full dotted path without needing the underscore dropped), `target`, `closing` (the tail
  both intercept laws must
  keep identical), then one law per file: `hold`, `orbit`, `pursuit`, `pronav`. Every law returns
  a `VelocityCommand` in local NED; the autopilot closes the inner velocity loop.
- `telemetry/` — `modes` is the MAVLink vocabulary both sides of the socket need and imports
  nothing; `tracker` is `TelemetryTracker`, a last-known-value view fed one message at a time.
  No pymavlink import either side, so parsing is tested against hand-built stub messages.
- `link.py` — `MavlinkLink`: commands, ACK/state-change confirmation, params, `send_velocity`.
  Deliberately **not** split: the command half calls the plumbing half throughout, and this is
  the one module allowed near a socket.
- `runner/` — `scheduler` (`FixedRateLoop` + `measure_sleep_overshoot`; reaches only for its own
  `stats` and `utils.percentile`), `flight` (`GuidanceRunner`), `protocols` (`GuidanceLaw`,
  `VehicleLink`), `stats` (`Tick`, `LoopStats`). The split follows the seam `tests/` already
  drew between the scheduling tests and the runner ones.
- `mission/` — `vehicle` (the `(link, tracker)` pair), `bringup` (`launch()`, `airborne()`),
  `reaction` (latency measurement — the only half that needs a guidance law).
- `utils/` — everything general, one admitted thing per file so `ls` shows what passed the test:
  `deadline` (the monotonic countdown every wait in `link`/`mission`/`commands` is built from),
  `command` (the argparse subcommand base), `percentiles` (shared by the loop and the analysis so
  neither imports the other), `text` (the CSV parsers), `emit` (`emit_json()` and its flag).
- `metrics/` (`events`, `summary`, `latency`), `recording/` (`sample`, `sinks`), `plots.py`,
  `commands/` — summaries, CSV log, figures, subcommands. `commands/__init__.py` deliberately
  re-exports **nothing**: a facade there makes importing one subcommand load all six, which drags
  `link` and pymavlink into `report`, the one subcommand that needs no vehicle. `cli.py` stays a
  **module**, not a package `__main__`, so the console script and every `python -m carrot_guide.cli`
  call site in the Makefile and `scripts/` keep working untouched.

### Invariants worth not breaking

- **Clock and sleep are injected.** `FixedRateLoop(monotonic=…, sleep=…)`,
  `GuidanceRunner(loop_factory=…)` and `MavlinkLink(monotonic=…, sleep=…)` exist so ten minutes
  of loop scheduling, or a retry loop that allows ninety seconds, is a millisecond-long test
  with a fake clock. `launch()` times itself off `link.monotonic` for the same reason. Keep new
  timing code parameterised the same way, and note that a fake connection has to advance the
  clock on a blocking read that finds nothing, or every wait built on it loops forever.
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

### Tests

`tests/carrot_guide/` mirrors `src/carrot_guide/` directory for directory, and the leaf under
each one is the kind of run: `guidance/unit/test_hold.py` and `guidance/integration/test_hold.py`
both test `guidance/hold.py`, one without a simulator and one against SITL. A module that is a
single file gets a directory of its own rather than sharing one — `cli/unit/test_cli.py` and
`cli/smoke/test_cli.py` for `cli.py`, `link/unit/test_link.py` for `link.py`.

The kinds are `unit` (the module or package the directory mirrors, reaching only into what it
already depends on — never up through `cli` or sideways to a peer), `component` (several modules
wired together in process: the runner closed around a fake vehicle, a subcommand run from argv,
still no simulator), `smoke` (the installed console script and `python -m carrot_guide.cli` as
subprocesses, failing rather than skipping when nothing is installed) and `integration` (a
running SITL). `make test` runs unit and component together, since both are fast; the markers
are what let `-m unit` alone answer whether the pieces stand up by themselves. Because the
mirror spreads every kind across the packages, a run is selected by marker rather than by path.
Every test carries its kind as a decorator — `@pytest.mark.unit`, or `@pytest.mark.integration`
above `@requires_simulator` — so what a test is is written on the test and not derived from
where it sits; `--strict-markers` in `pyproject.toml` turns a typo there into an error rather
than a test that belongs to no kind at all.

`tests/` and every directory under it is a **package**, because the mirror gives
`guidance/unit/test_hold.py` and `guidance/integration/test_hold.py` the same basename and only
a package makes those two modules. A helper sits beside the tests that use it —
**Every double lives in `tests/doubles/`**, named for what it stands in for, even the ones with
a single caller — a test file should hold tests: `messages` (a MAVLink message), `clocks` (a
clock that only moves when a test says so, and one where reading the time costs time so a
busy-wait terminates), `connection` (a pymavlink connection that answers like an autopilot, and
advances the fake clock on a blocking read that finds nothing) and `vehicles` (the two
`MavlinkLink` stand-ins: `FakeVehicle` flies its commands, `ColdVehicle` refuses them).

Other helpers sit beside the tests that use them: `guidance/toy_vehicle.py` (the point mass the
laws are integrated over) and `commands/parsers.py` (argv against one command alone, because its
options only exist once registered and `cli.build_parser()` would register all six). What
crosses packages goes higher still — `tests/runs.py` for synthetic sample series, and
`tests/simulator.py` for the url, `requires_simulator` and the `Flight` record. `tests/conftest.py`
holds the fixtures, including the session-scoped `flight`/`vehicle` pair that takes off once
for the whole run — up there
because the simulator tests are spread across packages and this is the only conftest above all
of them.

## Conventions

- Prose for humans (`README.md`, `SPEC.md`) is **Ukrainian**; all code, comments, docstrings, test
  names and commit messages are **English**. Keep that split.
- Comments are the exception, not the habit, and a file with none is fine. Docstrings are **not**
  required on modules, classes or functions — one that restates the name is worse than none.
  Exactly three things earn prose, in as few words as possible:
  1. a **measured value or its derivation**, without which the constant is a magic number;
  2. a line that **reads as wrong or arbitrary** unexplained — a workaround, an epsilon, a bitmask
     whose set bit means "ignore", a spin that must not `sleep(0)`;
  3. a **contract a caller could get wrong** and cannot see in the signature — returns `None` when
     there is no intercept, raises on an empty run, the result is always a value from the input.
- Everything else goes: no war stories about what a past bug did, no essays on why an alternative
  was rejected or what belongs in a module, no import-topology asides ("shared by two callers that
  must not import each other"), no restating the code, and no fact stated twice — keep the copy
  nearest the code. Prose was 31% of source lines and was cut on purpose; do not grow it back.
  When you shorten something, the measured numbers are what must survive.
- `cli.py`'s module docstring is `argparse`'s `description=__doc__`, so it *is* the text of
  `carrot-guide --help`. Editing it changes user-visible output.
- Tests are named as sentences (`test_a_dead_position_stream_stops_the_loop_rather_than_flying_blind`).
- No linter or formatter is configured; match surrounding style (~100 col, `from __future__ import
  annotations`, frozen dataclasses for values).
- Numbers in the README come from `docs/measurements/*.json`, produced by `scripts/measure.sh`.
  If a change moves the numbers, regenerate rather than editing the tables by hand. Summaries are
  written with `--summary`, not captured from stdout — under `memray` the profiler's banner lands
  on stdout around the JSON and silently corrupted two committed files that way.
- `scripts/plots.sh` is the single definition of the figures; `make plots` and `scripts/measure.sh`
  both call it, because two lists of plot commands had already drifted apart.
- **`utils/` holds everything general**, types and functions alike, and admits only what would
  read the same way *verbatim* in a project with no aircraft in it. Liftable is necessary but
  not sufficient: `FixedRateLoop` would lift cleanly and stays in `runner` anyway, because a
  10 Hz control loop is what this project *is*. `utils` takes what is incidental to the work —
  a countdown, a percentile, an argparse base — never the work itself. Anything that fails the
  test keeps its domain half at home: `Command` moved, the `--url` option group it used to carry
  stayed behind on `commands.VehicleCommand`. One admitted thing per file, so the package
  listing is the charter's own audit rather than something buried in a scroll.
- A subcommand is a `utils.Command` subclass in its own file under `commands/`, carrying its own
  options and its own handler, listed in `COMMANDS` in `cli.py`. Each rung of the ladder owns the
  options at its level and nothing else: `Command` is generic argparse plumbing and lives in
  `utils`; `VehicleCommand` adds `--url`/`--timeout` for the five that talk to a vehicle
  (`report` does not); the flying three subclass `FlightCommand`. Kept apart, `--kd` drifted onto
  the orbit parser and was read by nothing. Shared option groups belong on the rung that owns
  them, never as a loose module-level function a subclass has to remember to call. The ladder
  stays a class hierarchy — one file per rung, not a file per option group.

## Simulator

`docker-compose.yml` builds `docker/Dockerfile.sitl` from ArduPilot sources at the pinned tag
`Copter-4.5.7` (no usable prebuilt image exists; community ones are stale and amd64-only). Home is
Kyiv, MAVLink over TCP on `5760`. `SITL_SPEEDUP` must stay `1` for any timing measurement — the
guidance loop runs on the real clock, so speedup makes the measured rate meaningless. Wind is set
through parameters (`SIM_WIND_SPD`, `SIM_WIND_DIR`, `SIM_WIND_TURB`) from the run itself; steady
wind alone barely moves the error, turbulence is what makes a run worth quoting.
