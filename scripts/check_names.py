#!/usr/bin/env python3
# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
"""Find names a function uses but never defines.

Python resolves names at run time, so `python -m py_compile` is happy with a
function that reads a variable which does not exist; the error only appears when
that line is reached, which for a rarely-taken branch can be much later. Moving
code between scopes is exactly when this happens, and it did: an extraction left
`ov` referenced in main() and defined nowhere, and the compositor crash-looped.

Approximate by design. It walks each top-level function's own scope and the
module's, so a name that genuinely comes from elsewhere shows up as a false
positive; that is the right way round for a pre-commit check.
"""
from __future__ import annotations

import ast
import builtins
import sys


def _module_scope(tree: ast.Module) -> set[str]:
    names = set(dir(builtins))
    for node in ast.walk(tree):
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
            names.add(node.name)
        elif isinstance(node, ast.Import):
            for a in node.names:
                names.add((a.asname or a.name).split(".")[0])
        elif isinstance(node, ast.ImportFrom):
            for a in node.names:
                names.add(a.asname or a.name)
    for node in tree.body:
        if isinstance(node, ast.Assign):
            for t in node.targets:
                for x in ast.walk(t):
                    if isinstance(x, ast.Name):
                        names.add(x.id)
        elif isinstance(node, ast.AnnAssign) and isinstance(node.target, ast.Name):
            names.add(node.target.id)
    return names


def _bound(fn) -> set[str]:
    """Every name the function itself binds."""
    names = {a.arg for a in fn.args.args}
    names |= {a.arg for a in getattr(fn.args, "kwonlyargs", [])}
    for a in (fn.args.vararg, fn.args.kwarg):
        if a:
            names.add(a.arg)
    for node in ast.walk(fn):
        if isinstance(node, ast.Name) and isinstance(node.ctx, ast.Store):
            names.add(node.id)
        elif isinstance(node, ast.Lambda):
            names |= {a.arg for a in node.args.args}
        elif isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
            names.add(node.name)
            names |= {a.arg for a in getattr(getattr(node, "args", None), "args", [])}
        elif isinstance(node, ast.ExceptHandler) and node.name:
            names.add(node.name)
        elif isinstance(node, (ast.Import, ast.ImportFrom)):
            for a in node.names:
                names.add((a.asname or a.name).split(".")[0])
        elif isinstance(node, ast.NamedExpr) and isinstance(node.target, ast.Name):
            names.add(node.target.id)
        elif isinstance(node, (ast.For, ast.AsyncFor, ast.comprehension)):
            for x in ast.walk(node.target):
                if isinstance(x, ast.Name):
                    names.add(x.id)
        elif isinstance(node, ast.withitem) and node.optional_vars:
            for x in ast.walk(node.optional_vars):
                if isinstance(x, ast.Name):
                    names.add(x.id)
        elif isinstance(node, ast.Global) or isinstance(node, ast.Nonlocal):
            names |= set(node.names)
    return names


def check(path: str) -> list[str]:
    tree = ast.parse(open(path).read())
    problems = []

    def walk(node, enclosing: set[str]) -> None:
        for child in ast.iter_child_nodes(node):
            if isinstance(child, (ast.FunctionDef, ast.AsyncFunctionDef)):
                bound = _bound(child)
                used = {n.id for n in ast.walk(child)
                        if isinstance(n, ast.Name) and isinstance(n.ctx, ast.Load)}
                for name in sorted(used - bound - enclosing):
                    problems.append(f"{path}:{child.lineno}: {child.name}() "
                                    f"uses undefined {name!r}")
                walk(child, enclosing | bound)
            elif isinstance(child, ast.ClassDef):
                walk(child, enclosing | {child.name})
            else:
                walk(child, enclosing)

    walk(tree, _module_scope(tree))
    return problems


def check_before_assignment(path: str) -> list[str]:
    """Names read on a line before any line that assigns them.

    A different failure from an undefined name and invisible to the check above,
    which only asks whether a function binds a name at all. Moving code between
    scopes turns a caller's variable into a local one that is read on the first
    pass and only written on the second: `_last_blocked` did exactly that and
    raised UnboundLocalError on every frame.

    Line order is a crude proxy for control flow, so this only reports a name
    whose FIRST read is above its first write and which is never a parameter.
    Loops legitimately read what a later line assigns, so anything inside a loop
    is left alone.
    """
    tree = ast.parse(open(path).read())
    problems = []
    for fn in ast.walk(tree):
        if not isinstance(fn, (ast.FunctionDef, ast.AsyncFunctionDef)):
            continue
        params = {a.arg for a in fn.args.args}
        # A comprehension's variable is read to the left of where it is bound,
        # `x for x in xs`, so line order says nothing about it.
        for n in ast.walk(fn):
            if isinstance(n, ast.comprehension):
                params |= {x.id for x in ast.walk(n.target)
                           if isinstance(x, ast.Name)}
            elif isinstance(n, ast.Lambda):
                params |= {a.arg for a in n.args.args}
        loops = [(n.lineno, n.end_lineno) for n in ast.walk(fn)
                 if isinstance(n, (ast.For, ast.While, ast.AsyncFor))]
        first_read, first_write = {}, {}
        for n in ast.walk(fn):
            if not isinstance(n, ast.Name):
                continue
            book = first_write if isinstance(n.ctx, ast.Store) else first_read
            if n.id not in book or n.lineno < book[n.id]:
                book[n.id] = n.lineno
        for name, read_at in first_read.items():
            if name in params or name not in first_write:
                continue
            if read_at >= first_write[name]:
                continue
            if any(a <= read_at <= b for a, b in loops):
                continue
            problems.append(f"{path}:{read_at}: {fn.name}() reads {name!r} "
                            f"before assigning it on line {first_write[name]}")
    return problems


if __name__ == "__main__":
    found = [p for path in sys.argv[1:]
             for p in check(path) + check_before_assignment(path)]
    for line in found:
        print(line)
    print("no undefined names" if not found else f"{len(found)} problem(s)")
    sys.exit(1 if found else 0)
