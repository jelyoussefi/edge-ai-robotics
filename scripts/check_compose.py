#!/usr/bin/env python3
# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
"""Reject a compose file that repeats a key.

PyYAML keeps the last of two identical keys without complaining, so a plain
safe_load says the file is fine while Docker refuses it outright. This walks the
node tree instead of constructing it, which also sidesteps the merge tags the
compose file uses for its anchors.
"""
import sys

import yaml


def check(path):
    bad = []

    def walk(node, where=""):
        if isinstance(node, yaml.MappingNode):
            seen = {}
            for k, v in node.value:
                key = getattr(k, "value", None)
                if key in seen:
                    bad.append(f"{where}{key} (lines {seen[key]} and "
                               f"{k.start_mark.line + 1})")
                else:
                    seen[key] = k.start_mark.line + 1
                walk(v, f"{where}{key}.")
        elif isinstance(node, yaml.SequenceNode):
            for i, v in enumerate(node.value):
                walk(v, f"{where}{i}.")

    with open(path) as fh:
        walk(yaml.compose(fh))
    return bad


if __name__ == "__main__":
    problems = []
    for p in sys.argv[1:] or ["docker-compose.yml"]:
        for b in check(p):
            problems.append(f"{p}: duplicate key {b}")
    for line in problems:
        print(line)
    print("no duplicate keys" if not problems else "FIX THESE")
    sys.exit(1 if problems else 0)

