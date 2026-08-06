---
name: verify-stack
description: Run the project's three verifiers (py_compile, check_compose.py, check_names.py) across the repo and check docker-compose.yml against the Python os.environ.get fallbacks for tuning drift. Use before delivering a change, after a refactor that moved code between scopes, or whenever docker-compose.yml was edited.
---

This repo has no test suite and no CI. These three checkers plus running the demo *are* the validation gate, and `docs/reprise-edge-ai-robotics.md` requires all three before any delivery. Run them all even if only one file changed — `check_names.py` exists precisely because moving code between scopes breaks a file other than the one you edited.

## 1. Compile every module

```bash
python3 -m py_compile services/*/*.py services/*/**/*.py common/edgebot/*.py scripts/*.py
```

`make build` does **not** run this one. Syntax and indentation errors only.

## 2. Compose file

```bash
python3 scripts/check_compose.py docker-compose.yml
```

Rejects duplicate YAML keys. PyYAML keeps the last of two identical keys silently; Docker refuses the file outright. Needs PyYAML on the host. Exits 1 on failure.

## 3. Undefined names

```bash
python3 scripts/check_names.py services/*/*.py services/*/**/*.py common/edgebot/*.py
```

Walks each top-level function's scope plus the module's and reports names read but never defined, and reads-before-assignment. Python resolves names at run time, so `py_compile` is happy with a function referencing a variable that does not exist — the crash comes later, from a rarely-taken branch. This caught a real compositor crash-loop after an extraction left `ov` referenced and undefined.

**Approximate by design**: a name that genuinely comes from elsewhere (a star-import, an injected global) shows up as a false positive. Read each hit and judge it — do not assume every report is a bug, and do not silence it with a `noqa`. The repo has only four `noqa`s total, each justified in place.

## 4. Tuning-constant drift

`docker-compose.yml` is authoritative for every knob; the `os.environ.get(NAME, default)` fallback in the service is a safety net. They have drifted before (`RETURN_TO`, `ROBOT_HEIGHT`, `ROI_MARGIN`, `RENDER_SCALE`).

For each `${NAME:-default}` in `docker-compose.yml`, find the matching `os.environ.get("NAME", ...)` in the service source and compare the two defaults. Report every mismatch as `NAME: compose=<x> code=<y> (services/<svc>/<file>.py:<line>)`. Do not silently "fix" them — compose wins, but changing a live tuning value is a behavioural change the user decides on.

Also verify the load-bearing relationships still hold, since a compose edit can break them without any checker noticing:

- `OBSTACLE_STALE` (sim) **>** `ROI_PERIOD` (compositor) — otherwise footprints expire between messages and the robot ping-pongs.
- `OBSTACLE_MARGIN` identical in `compositor` and `groundfloor` — otherwise the two footprint sets aren't comparable and `make suite-compare` is meaningless.

## Reporting

Report pass/fail per step with the actual command output for anything that failed. If everything passes, say so plainly and note that this covers static checks only — it does not prove the demo runs. Behavioural confirmation still needs `make run` on the board, and `make seg-test` (exits 1 when nothing is detected) for the segmentation path.
