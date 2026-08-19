PYTHON ?= python3
ifeq ($(wildcard .env),)
$(error .env is missing: cp .env.example .env, then set CARROT_GUIDE_VENV to an absolute path)
endif
include .env
ifeq ($(strip $(CARROT_GUIDE_VENV)),)
$(error CARROT_GUIDE_VENV is empty in .env; without it BIN would collapse to /bin)
endif
VENV := $(CARROT_GUIDE_VENV)
BIN := $(VENV)/bin
SITL_URL ?= tcp:127.0.0.1:5760
SITL_HOST ?= 127.0.0.1
SITL_PORT ?= 5760

.PHONY: venv test smoke check test-sitl sitl-up sitl-down hold orbit latency profile plots measure clean

venv:
	$(PYTHON) -m venv $(VENV)
	$(BIN)/pip install -q --upgrade pip
	$(BIN)/pip install -q -e '.[plots,dev]'

test:
	$(BIN)/pytest -q -m 'not sitl'

# The unit tests import from src/ directly (pythonpath in pyproject.toml), so they stay
# green even when the installed console script is broken — which is exactly what a stale
# editable install looks like after the checkout is moved. One line closes that gap.
smoke:
	$(BIN)/carrot-guide --help >/dev/null

# Needs the simulator up: make sitl-up
test-sitl:
	CARROT_SITL_URL=$(SITL_URL) $(BIN)/pytest -q -m sitl

# The whole suite in one step. The port answers before the autopilot is ready to fly,
# but that part the tests wait out themselves; what they cannot survive is connecting
# to a socket nobody is listening on yet.
check: test smoke sitl-up
	@printf 'waiting for the simulator on %s:%s' $(SITL_HOST) $(SITL_PORT)
	@for _ in $$(seq 1 60); do \
		nc -z $(SITL_HOST) $(SITL_PORT) 2>/dev/null && break; \
		printf '.'; sleep 1; \
	done; echo
	@# The loop above cannot tell "the port opened" from "sixty seconds went by", and
	@# a simulator that never started would otherwise be reported as a connection
	@# error from pymavlink rather than as a simulator that never started.
	@nc -z $(SITL_HOST) $(SITL_PORT) 2>/dev/null || \
		{ echo 'the simulator never came up on $(SITL_HOST):$(SITL_PORT)'; exit 1; }
	@$(MAKE) test-sitl

sitl-up:
	docker compose up -d --build sitl

sitl-down:
	docker compose down

hold:
	$(BIN)/carrot-guide hold --url $(SITL_URL) --north 25 --altitude 15 \
		--wind 6 --turbulence 1 --seconds 60

orbit:
	$(BIN)/carrot-guide orbit --url $(SITL_URL) --radius 25 --speed 4 --altitude 20 \
		--wind 6 --turbulence 1 --lookahead 0.22 --seconds 90

latency:
	$(BIN)/carrot-guide latency --url $(SITL_URL) --trials 6

# Ten minutes of flying under memray, to see whether the loop leaks.
profile:
	$(BIN)/memray run --force -o memray-hold-600.bin -m carrot_guide.cli hold \
		--url $(SITL_URL) --north 25 --altitude 15 --wind 6 --turbulence 1 --seconds 600
	$(BIN)/memray stats memray-hold-600.bin

# Needs the logs a run leaves behind; logs/ is not in the repository, so this is a
# post-`make measure` step rather than something a fresh clone can do.
plots:
	scripts/plots.sh

# Everything the README quotes, in one pass.
measure:
	SITL_URL=$(SITL_URL) scripts/measure.sh

clean:
	rm -rf logs memray-*.bin memray-*.html
