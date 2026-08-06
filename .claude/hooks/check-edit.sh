#!/usr/bin/env bash
# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
#
# PostToolUse hook: run this project's own verifiers on the file that was just
# edited, instead of waiting for `make build` to run them at image-build time.
#
# check_names.py exists because py_compile is happy with a function that reads a
# name defined nowhere -- the crash only surfaces when that branch is reached,
# which for a rarely-taken branch can be much later. Catching it at the edit is
# the whole point; catching it at build time already cost a compositor
# crash-loop once.
#
# Reports back through additionalContext rather than blocking: check_names.py is
# approximate by design (a name that genuinely comes from elsewhere reads as a
# false positive), so a hard block would fight the developer over hits that are
# fine. Claude sees the output and decides.
set -uo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

# Some services/ subdirectories have a root-owned __pycache__ left by container
# builds, so py_compile cannot write next to the source. Park the bytecode
# elsewhere; we only care about the exit status.
export PYTHONPYCACHEPREFIX="${TMPDIR:-/tmp}/edgebot-pycache"

file="$(jq -r '.tool_response.filePath // .tool_input.file_path // empty')"
[[ -n "$file" && -f "$file" ]] || exit 0

out=""
case "$file" in
  "$root"/services/*.py | "$root"/common/*.py | "$root"/scripts/*.py)
    out="$(python3 -m py_compile "$file" 2>&1)" || {
      printf '%s' "$out" | jq -Rs \
        '{hookSpecificOutput:{hookEventName:"PostToolUse",additionalContext:("py_compile failed:\n"+.)}}'
      exit 0
    }
    out="$(python3 "$root/scripts/check_names.py" "$file" 2>&1)" || {
      printf '%s' "$out" | jq -Rs \
        '{hookSpecificOutput:{hookEventName:"PostToolUse",additionalContext:("check_names.py flagged names used but never defined (approximate -- judge each hit, do not add noqa):\n"+.)}}'
      exit 0
    }
    ;;
  */docker-compose.yml | */docker-compose.yaml)
    out="$(python3 "$root/scripts/check_compose.py" "$file" 2>&1)" || {
      printf '%s' "$out" | jq -Rs \
        '{hookSpecificOutput:{hookEventName:"PostToolUse",additionalContext:("check_compose.py failed -- Docker will refuse this file:\n"+.)}}'
      exit 0
    }
    ;;
esac

exit 0
