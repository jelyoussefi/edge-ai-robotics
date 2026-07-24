# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
"""Keyboard control.

Runs in a terminal with the tty in raw mode, so no X connection and no access to
/dev/input is needed. Each keypress nudges a velocity target which then decays
back to zero, giving a hold-to-move feel from discrete key events.
"""

from __future__ import annotations

import os
import signal
import sys
import termios
import time
import tty

from edgebot import topics
from edgebot.bus import Publisher

RATE_HZ = 50.0
DECAY_PER_S = 1.6  # how fast the command falls back to zero when keys are idle
STEP_V = 0.25
STEP_W = 0.35

BINDINGS = """
  w / s      walk forward / backward
  a / d      turn left / right
  q / e      strafe left / right
  space      stop immediately
  r          reset the robot to its start pose
  ctrl-c     quit
"""


class RawTerminal:
    def __enter__(self) -> "RawTerminal":
        self.fd = sys.stdin.fileno()
        self.saved = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        os.set_blocking(self.fd, False)
        return self

    def __exit__(self, *_: object) -> None:
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.saved)

    @staticmethod
    def read() -> str:
        return sys.stdin.read(1) or ""


def clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def decay(value: float, amount: float) -> float:
    """Move a value toward zero by at most `amount`, never past it."""
    if abs(value) <= amount:
        return 0.0
    return value - amount if value > 0 else value + amount


def main() -> None:
    pub = Publisher()
    vx = vy = wz = 0.0
    running = True

    def stop(*_: object) -> None:
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    print("Edge AI Robotics — keyboard control")
    print(BINDINGS)

    dt = 1.0 / RATE_HZ
    with RawTerminal() as term:
        while running:
            loop_start = time.perf_counter()

            while (key := term.read()):
                if key == "w":
                    vx = clamp(vx + STEP_V, topics.MAX_VX)
                elif key == "s":
                    vx = clamp(vx - STEP_V, topics.MAX_VX)
                elif key == "a":
                    wz = clamp(wz + STEP_W, topics.MAX_WZ)
                elif key == "d":
                    wz = clamp(wz - STEP_W, topics.MAX_WZ)
                elif key == "q":
                    vy = clamp(vy + STEP_V, topics.MAX_VY)
                elif key == "e":
                    vy = clamp(vy - STEP_V, topics.MAX_VY)
                elif key == " ":
                    vx = vy = wz = 0.0
                elif key == "r":
                    pub.send(topics.CMD_VEL, {"vx": 0.0, "vy": 0.0, "wz": 0.0, "reset": True})
                    vx = vy = wz = 0.0
                elif key == "\x03":
                    running = False

            step = DECAY_PER_S * dt
            vx, vy, wz = decay(vx, step), decay(vy, step), decay(wz, step)

            pub.send(topics.CMD_VEL, {"vx": vx, "vy": vy, "wz": wz, "stamp": time.time()})

            sys.stdout.write(f"\r  vx {vx:+.2f}  vy {vy:+.2f}  wz {wz:+.2f}   ")
            sys.stdout.flush()

            time.sleep(max(0.0, dt - (time.perf_counter() - loop_start)))

    pub.send(topics.CMD_VEL, {"vx": 0.0, "vy": 0.0, "wz": 0.0, "stamp": time.time()})
    pub.close()
    print("\nstopped")


if __name__ == "__main__":
    main()
