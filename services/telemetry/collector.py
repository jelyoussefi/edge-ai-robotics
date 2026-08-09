#!/usr/bin/env python3
# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
"""What the CPU, the iGPU and the NPU are doing, straight from sysfs.

Read from the kernel rather than from a tool. `qmassa` and `intel_gpu_top` both
exist and both read exactly the files below; going through one of them would add
a binary to the image, a parser for its output format, and a second thing that
can be absent, in exchange for nothing this needs.

**Every figure is either a number or None, and None means the platform does not
expose it.** On this board that is true of iGPU power and of NPU power: the xe
driver registers no hwmon node here (`/sys/class/hwmon` holds acpi_fan, acpitz,
nvme, ucsi, asus, coretemp and iwlwifi, and nothing for the GPU), and the
accel/NPU driver exposes busy time, frequency and memory but no energy counter.
Publishing 0 W for either would be indistinguishable from a genuinely idle
engine, so they go out as null and the console prints "not exposed by this
driver". A missing measurement should look missing.

Everything here is a COUNTER that has to be differenced -- jiffies, idle
residency, busy microseconds -- so the first tick after start has nothing to
compare against and is skipped rather than published as zero.

What has to be mounted, all read-only:
  /proc/stat                  CPU jiffies (already in the container's own /proc)
  /sys/class/powercap         RAPL energy counters, root-only files
  /sys/class/drm              xe GT idle residency and frequency
  /sys/class/accel            NPU busy time, frequency, memory
RAPL's energy_uj is mode 0400 root:root -- readable here only because the
container runs as root, and unreadable to an unprivileged user on the host.
"""
from __future__ import annotations

import logging
import os
import signal
import time

from edgebot import topics
from edgebot.bus import Publisher

log = logging.getLogger("telemetry")

PERIOD = float(os.environ.get("TELEMETRY_PERIOD", "1.0"))
POWERCAP = os.environ.get("POWERCAP_ROOT", "/sys/class/powercap")
DRM = os.environ.get("DRM_ROOT", "/sys/class/drm")
ACCEL = os.environ.get("ACCEL_ROOT", "/sys/class/accel")

running = True


def _stop(_signum, _frame) -> None:
    global running
    running = False


# Why a figure is missing, keyed by the field name the console shows. The
# distinction is load-bearing: "the driver does not expose this" is a fact about
# the hardware and stays true after a restart, while "the container runtime
# blocked it" is a fact about how we launched and is fixable in compose. A panel
# that renders both as the same grey box would send someone hunting for a
# missing sensor that is actually right there behind an AppArmor rule.
UNAVAILABLE: dict[str, str] = {}

NOT_EXPOSED = "not exposed by this driver"
RUNTIME_BLOCKED = ("blocked by the container runtime -- the attribute exists "
                   "but reading it was denied (see the security_opt notes on "
                   "the telemetry service)")


def _read(path: str) -> str | None:
    try:
        with open(path) as fh:
            return fh.read().strip()
    except PermissionError:
        return None
    except (OSError, ValueError):
        return None


def _why_unreadable(path: str) -> str:
    """Tell a denied read apart from an absent attribute."""
    try:
        with open(path):
            return ""
    except PermissionError:
        return RUNTIME_BLOCKED
    except OSError:
        return NOT_EXPOSED


def _read_float(path: str) -> float | None:
    raw = _read(path)
    if raw is None:
        return None
    try:
        return float(raw)
    except ValueError:
        return None


class CpuLoad:
    """Aggregate and per-core busy fraction from /proc/stat."""

    def __init__(self) -> None:
        self._prev: dict[str, tuple[float, float]] = {}

    def sample(self) -> tuple[float | None, list[float]]:
        raw = _read("/proc/stat")
        if raw is None:
            return None, []
        total_pct: float | None = None
        cores: list[tuple[int, float]] = []
        for line in raw.splitlines():
            if not line.startswith("cpu"):
                break
            parts = line.split()
            key = parts[0]
            vals = [float(v) for v in parts[1:]]
            # user nice system idle iowait irq softirq steal ...
            # idle time is idle + iowait: a core waiting on IO is not working.
            idle = vals[3] + (vals[4] if len(vals) > 4 else 0.0)
            total = sum(vals)
            prev = self._prev.get(key)
            self._prev[key] = (idle, total)
            if prev is None:
                continue
            d_idle, d_total = idle - prev[0], total - prev[1]
            if d_total <= 0:
                continue
            pct = 100.0 * (1.0 - d_idle / d_total)
            if key == "cpu":
                total_pct = pct
            else:
                cores.append((int(key[3:]), pct))
        cores.sort()
        return total_pct, [round(p, 1) for _, p in cores]


class Rapl:
    """Package/core/uncore/dram/psys power, by differencing energy counters.

    The counters wrap at max_energy_range_uj. A wrap looks like a large negative
    delta, which would render as a huge negative power spike; it is folded back
    rather than clamped to zero so the average over the wrap stays right.
    """

    def __init__(self) -> None:
        self.blocked = ""
        self.domains: dict[str, tuple[str, float]] = {}
        for entry in sorted(os.listdir(POWERCAP)) if os.path.isdir(POWERCAP) else []:
            if not entry.startswith("intel-rapl:"):
                continue
            base = os.path.join(POWERCAP, entry)
            name = _read(os.path.join(base, "name"))
            energy = os.path.join(base, "energy_uj")
            if not name:
                continue
            if _read_float(energy) is None:
                self.blocked = self.blocked or _why_unreadable(energy)
                continue
            rng = _read_float(os.path.join(base, "max_energy_range_uj")) or 0.0
            self.domains[name] = (energy, rng)
        if not self.domains:
            reason = self.blocked or NOT_EXPOSED
            for field in ("pkg_w", "core_w", "uncore_w", "dram_w", "psys_w"):
                UNAVAILABLE[field] = reason
            log.warning("no readable RAPL domain under %s: %s", POWERCAP, reason)
        else:
            log.info("RAPL domains: %s", ", ".join(sorted(self.domains)))
        self._prev: dict[str, tuple[float, float]] = {}

    def sample(self) -> dict[str, float | None]:
        out: dict[str, float | None] = {}
        now = time.monotonic()
        for name, (path, rng) in self.domains.items():
            uj = _read_float(path)
            if uj is None:
                out[name] = None
                continue
            prev = self._prev.get(name)
            self._prev[name] = (uj, now)
            if prev is None:
                out[name] = None
                continue
            d_uj, dt = uj - prev[0], now - prev[1]
            if d_uj < 0 and rng > 0:
                d_uj += rng
            out[name] = round(d_uj / 1e6 / dt, 2) if dt > 0 else None
        return out


class Gpu:
    """Intel xe: busy from GT idle residency, frequency from freq0.

    The driver reports how long the GT spent IDLE, cumulatively, in ms. Busy is
    the complement over the interval. There is no busy counter to read directly
    and no hwmon power node on this board, which is why gpu_w is always None
    here rather than sometimes None.
    """

    def __init__(self) -> None:
        self.idle_path = None
        self.freq_path = None
        self.freq_req_path = None
        self.power_path = None
        for card in sorted(os.listdir(DRM)) if os.path.isdir(DRM) else []:
            gt = os.path.join(DRM, card, "device", "tile0", "gt0")
            if not os.path.isdir(gt):
                continue
            idle = os.path.join(gt, "gtidle", "idle_residency_ms")
            if _read_float(idle) is not None:
                self.idle_path = idle
                self.freq_path = os.path.join(gt, "freq0", "act_freq")
                self.freq_req_path = os.path.join(gt, "freq0", "cur_freq")
            break
        # hwmon under the card, if this driver ever grows one. Looked up rather
        # than assumed absent, so a driver update starts reporting power with no
        # code change -- and so "not exposed" stays a measurement, not a belief.
        for card in sorted(os.listdir(DRM)) if os.path.isdir(DRM) else []:
            hw = os.path.join(DRM, card, "device", "hwmon")
            for node in sorted(os.listdir(hw)) if os.path.isdir(hw) else []:
                for attr in ("power1_average", "energy1_input"):
                    p = os.path.join(hw, node, attr)
                    if _read_float(p) is not None:
                        self.power_path = p
                        break
        if self.idle_path is None:
            UNAVAILABLE["gpu_pct"] = NOT_EXPOSED
            UNAVAILABLE["gpu_mhz"] = NOT_EXPOSED
            log.warning("no xe GT idle counter under %s: GPU load unavailable",
                        DRM)
        if self.power_path is None:
            UNAVAILABLE["gpu_w"] = NOT_EXPOSED
            log.info("no GPU hwmon power node: GPU power is %s", NOT_EXPOSED)
        self._prev: tuple[float, float] | None = None

    def sample(self):
        # act_freq is the frequency the GT is ACTUALLY at, and it reads 0
        # whenever the tile is gated at the sampling instant -- which is
        # common even at 40 % busy, because the busy figure is an average
        # over the second and this is a spot reading. Both go out: the
        # gauge shows the achieved one and the requested one behind it,
        # rather than one number that quietly means two different things.
        mhz = _read_float(self.freq_path) if self.freq_path else None
        req = _read_float(self.freq_req_path) if self.freq_req_path else None
        idle_ms = _read_float(self.idle_path) if self.idle_path else None
        pct = None
        if idle_ms is not None:
            now = time.monotonic()
            if self._prev is not None:
                d_idle = idle_ms - self._prev[0]
                dt_ms = (now - self._prev[1]) * 1000.0
                if dt_ms > 0:
                    pct = round(max(0.0, min(100.0,
                                             100.0 * (1.0 - d_idle / dt_ms))), 1)
            self._prev = (idle_ms, now)
        watts = None
        if self.power_path:
            v = _read_float(self.power_path)
            # power1_average is microwatts; energy1_input would need
            # differencing, which is why only the former is used directly.
            if v is not None and self.power_path.endswith("power1_average"):
                watts = round(v / 1e6, 2)
        return pct, mhz, watts, req


class Npu:
    """Intel NPU (accel): busy microseconds, frequency, memory.

    npu_busy_time_us is cumulative across all contexts, so the busy fraction is
    its delta over the interval. No energy counter exists on this driver.
    """

    def __init__(self) -> None:
        self.base = None
        for entry in sorted(os.listdir(ACCEL)) if os.path.isdir(ACCEL) else []:
            base = os.path.join(ACCEL, entry, "device")
            if _read_float(os.path.join(base, "npu_busy_time_us")) is not None:
                self.base = base
                break
        UNAVAILABLE["npu_w"] = NOT_EXPOSED
        if self.base is None:
            for field in ("npu_pct", "npu_mhz", "npu_mem_mb"):
                UNAVAILABLE[field] = NOT_EXPOSED
            log.warning("no NPU busy counter under %s: NPU load unavailable",
                        ACCEL)
        else:
            log.info("NPU at %s (power: %s)", self.base, NOT_EXPOSED)
        self._prev: tuple[float, float] | None = None

    def sample(self):
        if self.base is None:
            return None, None, None, None
        busy_us = _read_float(os.path.join(self.base, "npu_busy_time_us"))
        mhz = _read_float(os.path.join(self.base, "npu_current_frequency_mhz"))
        mem = _read_float(os.path.join(self.base, "npu_memory_utilization"))
        pct = None
        if busy_us is not None:
            now = time.monotonic()
            if self._prev is not None:
                d_busy = busy_us - self._prev[0]
                dt_us = (now - self._prev[1]) * 1e6
                if dt_us > 0:
                    pct = round(max(0.0, min(100.0, 100.0 * d_busy / dt_us)), 1)
            self._prev = (busy_us, now)
        return pct, mhz, None, (round(mem / 1e6, 1) if mem is not None else None)


def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(name)s %(levelname)s %(message)s")
    signal.signal(signal.SIGTERM, _stop)
    signal.signal(signal.SIGINT, _stop)

    pub = Publisher()
    cpu, rapl, gpu, npu = CpuLoad(), Rapl(), Gpu(), Npu()
    sources = {
        "cpu": "/proc/stat",
        "power": POWERCAP + "/intel-rapl:*/energy_uj",
        "gpu_busy": gpu.idle_path or "(absent)",
        "gpu_freq": gpu.freq_path or "(absent)",
        "gpu_power": gpu.power_path or "(not exposed by this driver)",
        "npu": (npu.base or "(absent)") + "/npu_busy_time_us",
        "npu_power": "(not exposed by this driver)",
    }
    log.info("publishing %s every %.1f s", topics.PLATFORM, PERIOD)

    first = True
    while running:
        t0 = time.monotonic()
        cpu_pct, cores = cpu.sample()
        power = rapl.sample()
        gpu_pct, gpu_mhz, gpu_w, gpu_mhz_req = gpu.sample()
        npu_pct, npu_mhz, npu_w, npu_mem = npu.sample()
        # The first tick has no previous counter to difference against. Skipping
        # it costs one second and avoids opening every gauge at a flat zero that
        # looks like a reading.
        if not first:
            pub.send(topics.PLATFORM, {
                "cpu_pct": cpu_pct, "cpu_per_core": cores,
                "pkg_w": power.get("package-0"), "core_w": power.get("core"),
                "uncore_w": power.get("uncore"), "dram_w": power.get("dram"),
                "psys_w": power.get("psys"),
                "gpu_pct": gpu_pct, "gpu_mhz": gpu_mhz,
                "gpu_mhz_req": gpu_mhz_req, "gpu_w": gpu_w,
                "npu_pct": npu_pct, "npu_mhz": npu_mhz, "npu_w": npu_w,
                "npu_mem_mb": npu_mem,
                "sources": sources, "unavailable": UNAVAILABLE,
                "stamp": time.time(),
            })
        first = False
        time.sleep(max(0.05, PERIOD - (time.monotonic() - t0)))

    pub.close()


if __name__ == "__main__":
    main()
