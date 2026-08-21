"""Everything general this project needs that has nothing to do with flying.

The test for what belongs here is deliberately narrow: code that would read the same
way, verbatim, in a project with no aircraft in it — liftable whole, with nothing from
here needed to explain it. A countdown on the monotonic clock, an argparse subcommand,
a percentile, the inverse of `str(bool)`, a JSON summary written where nothing else can
print over it.

That is stricter than "mentions no vehicle", and it is not the only question. Being
liftable is necessary, not sufficient: `runner.FixedRateLoop` would lift cleanly too,
and it stays in `runner`, because a 10 Hz control loop is what this project *is*. What
belongs here is incidental to the work — a countdown, a percentile, an argparse base —
not the work itself. Anything failing the test keeps its domain half at home: `Command`
is here, the `--url` option group it used to carry is not.

Stated because a module named `utils` otherwise becomes the place things are put when
nobody wants to decide where they go, and then everything imports everything.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Sequence, get_type_hints


@dataclass(frozen=True)
class Deadline:
    """An instant to work towards, on the monotonic clock.

    Nearly every wait in the transport and mission layers is the same shape: keep
    trying until an instant, and never block for more than a slice of what is left.
    Spelled out inline that was five copies of

        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            ...
        do_something(timeout=min(0.5, remaining))

    each with its own literal cap and its own failure message, so checking that they
    agreed meant diffing them. Naming the countdown puts the arithmetic in one place
    and leaves the call sites saying what they are waiting for.

    Frozen because this is an instant, not a timer: `remaining_s` reads differently
    each time because the clock moved, never because the object did. It also means a
    callee handed one cannot re-base its caller's budget — `mission.launch` gives the
    same deadline to three separate calls, and `slice()` can only ever narrow it.

    The clock is a parameter for the same reason it is one in `runner.FixedRateLoop`:
    with it injected, a timeout is a test that runs in microseconds rather than a test
    that waits.
    """

    at_s: float
    monotonic: Callable[[], float] = time.monotonic

    @classmethod
    def after(cls, seconds: float, monotonic: Callable[[], float] = time.monotonic) -> "Deadline":
        return cls(monotonic() + seconds, monotonic)

    @property
    def remaining_s(self) -> float:
        """Time left, negative once the deadline has passed."""
        return self.at_s - self.monotonic()

    @property
    def expired(self) -> bool:
        return self.remaining_s <= 0.0

    def slice(self, cap_s: float) -> float:
        """At most `cap_s`, never past the deadline, and never negative.

        The floor matters: a caller that hands this straight to a blocking read would
        otherwise pass a negative timeout — which the socket layer reads as "no wait"
        or as "wait forever" depending on which one it is, neither being what a lapsed
        deadline means.
        """
        return max(0.0, min(cap_s, self.remaining_s))


def percentile(values: Sequence[float], fraction: float) -> float:
    """Nearest-rank percentile — no interpolation between neighbouring samples.

    Deliberately the simplest definition: every number this returns is a value the run
    actually produced, which is what makes a quoted p95 checkable against the log.

    Not `statistics.quantiles`, which exists and was considered. It interpolates, so its
    answer sits *between* two samples and cannot be found in the log the README quotes it
    from — on `logs/hold-gusty.csv` it gives 17.042 m where the sample at that rank in the
    run is 17.018 m, and the default `method="exclusive"` says 17.473 m. It also
    raises on fewer than two data points, and a loop that never ticked has to summarise
    rather than crash (see `runner.LoopStats`). `numpy.percentile` interpolates the same
    way and is not a dependency here.

    Shared by two callers that must not import each other: `runner` quotes the loop's
    lateness percentile while a run is in the air, and `metrics` quotes the error
    percentile long afterwards, and the flying path must not drag in the analysis layer
    (see `metrics` for why).
    """
    if not values:
        return 0.0
    ordered = sorted(values)
    index = min(len(ordered) - 1, int(round(fraction * (len(ordered) - 1))))
    return ordered[index]


def parse_bool(text: str) -> bool:
    """Inverse of `str(bool)` — the only true value is `True`.

    Spelled out rather than left to `bool(text)`, which is also true for the string
    `"False"`; reading a CSV column back that way makes every false row true.
    """
    return text == "True"


# A field annotated with anything not in here raises, which is the right moment for it:
# the alternative is a column that reads back as a string and poisons what follows.
TEXT_PARSER_BY_TYPE: dict[type, Callable[[str], object]] = {
    float: float,
    str: str,
    bool: parse_bool,
}


def parsers_for(cls: type) -> dict[str, Callable[[str], object]]:
    """One text parser per field of a dataclass, chosen by its declared type.

    `get_type_hints` is what resolves those annotations to real types: under `from
    __future__ import annotations` a field's `.type` is a plain string, and a table
    matching on the spelling of an annotation stops working the day one is written
    differently — it does not fail, it silently stops recognising the column.
    """
    return {name: TEXT_PARSER_BY_TYPE[hint] for name, hint in get_type_hints(cls).items()}


@dataclass(frozen=True)
class Command:
    """One subcommand of a command-line program: its name, and what it does.

    Knows how to put itself on an `argparse` parser and nothing else — what options a
    particular subcommand takes, and what it does with them, are the subclass's. Kept
    as a class per subcommand rather than a pile of `add_argument` calls because, run
    together, `--kd` drifted onto the parser of a law that never reads it and was
    silently accepted for the length of a measurement campaign.
    """

    name: str
    help: str

    def add_arguments(self, sub: argparse.ArgumentParser) -> None:
        raise NotImplementedError

    def run(self, args: argparse.Namespace) -> int:
        raise NotImplementedError

    def register(self, subparsers: argparse._SubParsersAction) -> None:
        sub = subparsers.add_parser(self.name, help=self.help)
        self.add_arguments(sub)
        sub.set_defaults(handler=self.run)

    @staticmethod
    def _add_summary(sub: argparse.ArgumentParser) -> None:
        """The `--summary` flag that `emit_json` writes to; see there for why it exists.

        On `Command` rather than loose beside it, and rather than one rung lower: every
        subcommand that emits a summary is a `Command`, and the one that is *only* a
        `Command` — `report` — is among them.
        """
        sub.add_argument(
            "--summary",
            default=None,
            help="also write the JSON summary here, out of reach of anything else on stdout",
        )


def emit_json(payload: dict[str, Any], path: str | None = None) -> None:
    """Print a summary as JSON, and write it somewhere exact if asked to.

    Two destinations rather than one because capturing stdout is not safe enough for an
    artefact something else quotes: any tool wrapping the process can print around the
    JSON and leave what was captured unparseable. It is not hypothetical — run under
    `memray`, the profiler's banner landed either side of the payload and two of this
    project's committed measurement files were silently corrupt that way. Writing the
    file directly puts the summary out of reach of anything else that prints.
    """
    text = json.dumps(payload, indent=2, ensure_ascii=False)
    sys.stdout.write(text + "\n")
    if path:
        target = Path(path)
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(text + "\n", encoding="utf-8")
