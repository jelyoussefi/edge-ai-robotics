#!/usr/bin/env python3
# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
"""What the CPU, the iGPU and the NPU are doing.

Power comes from **Intel PCM**, reading RAPL energy over the MSRs, by the method
in reference/intel-toolkit/system_utils.py::get_power_usage: run `pcm 1 -csv`
for one interval, parse the single data row, and take joules-per-second as
watts. The counter priority is that reference's: Proc + DRAM > System > CPU. On
this board PCM offers `Proc Energy (Joules)`, `Power Plane 0/1` and `SYSTEM
Energy` but no DRAM column, so Proc is used alone.

GPU busy, frequency and power come from **qmassa**, which reports per-engine
busy -- the maximum across render/blitter/video/compute, which is what xpu-smi
and intel_gpu_top mean by "the GPU is busy" -- plus `gpu_cur_power`.

THE PMU LOCK IS LOAD-BEARING. PCM and qmassa both claim hardware PMU counters
while initialising and the kernel allows one client at a time; whichever starts
second fails. They run on two separate threads here -- neither is fast enough to
sit inside a 1 Hz loop, PCM taking about 1.3 s and qmassa about 2 s -- so the
lock is not a formality, it is the only thing keeping them apart. It is held for
the subprocess call alone; parsing happens after it is released.

Cheap readings (CPU jiffies, NPU sysfs) stay on the main loop and publish every
second. Power and GPU carry whatever the workers last produced, so those two are
a few seconds old; each ships its own age rather than pretending otherwise.

Every figure is a number or None, and None NEVER means zero. Two reasons are
kept apart because they are different problems:
  "not exposed by this driver"       a fact about the hardware
  "blocked by the container runtime" a fact about how we launched, and fixable
On this board the only genuinely unexposed figure is NPU power: the accel driver
publishes busy time, frequency and memory but no energy counter. iGPU power IS
available -- through qmassa, not through /sys/class/hwmon, which is where an
earlier version of this file looked before wrongly concluding otherwise.
"""
from __future__ import annotations

import glob
import json
import logging
import os
import re
import signal
import subprocess
import tempfile
import threading
import time

from edgebot import topics
from edgebot.bus import Publisher

log = logging.getLogger("telemetry")

PERIOD = float(os.environ.get("TELEMETRY_PERIOD", "1.0"))
ACCEL = os.environ.get("ACCEL_ROOT", "/sys/class/accel")
DRM = os.environ.get("DRM_ROOT", "/sys/class/drm")
HWMON = os.environ.get("HWMON_ROOT", "/sys/class/hwmon")
PCM_BIN = os.environ.get("PCM_BIN", "/usr/local/sbin/pcm")
QMASSA_BIN = os.environ.get("QMASSA_BIN", "/usr/local/bin/qmassa")
# How often each PMU tool re-samples. Much below 2 s the two of them plus the
# lock keep the PMU busy continuously, which buys no resolution and burns a core.
PMU_PERIOD = float(os.environ.get("PMU_PERIOD", "2.0"))

NOT_EXPOSED = "not exposed by this driver"
RUNTIME_BLOCKED = ("blocked by the container runtime -- see the cap_add, "
                   "device_cgroup_rules and security_opt notes on the "
                   "telemetry service")

UNAVAILABLE: dict[str, str] = {}

# One PMU client at a time. See the module docstring.
_PMU_LOCK = threading.Lock()

running = True


def _stop(_signum, _frame) -> None:
    global running
    running = False


def _read(path: str) -> str | None:
    try:
        with open(path) as fh:
            return fh.read().strip()
    except OSError:
        return None


def _read_float(path: str) -> float | None:
    raw = _read(path)
    try:
        return float(raw) if raw is not None else None
    except ValueError:
        return None


# ---------------------------------------------------------------- CPU load

class CpuLoad:
    """Aggregate and per-core busy fraction from /proc/stat."""

    def __init__(self) -> None:
        self._prev: dict[str, tuple[float, float]] = {}

    def sample(self) -> tuple[float | None, list[float]]:
        raw = _read("/proc/stat")
        if raw is None:
            return None, []
        total: float | None = None
        cores: list[tuple[int, float]] = []
        for line in raw.splitlines():
            if not line.startswith("cpu"):
                break
            parts = line.split()
            key, vals = parts[0], [float(v) for v in parts[1:]]
            # idle + iowait: a core waiting on IO is not doing work.
            idle = vals[3] + (vals[4] if len(vals) > 4 else 0.0)
            tot = sum(vals)
            prev = self._prev.get(key)
            self._prev[key] = (idle, tot)
            if prev is None:
                continue
            d_idle, d_tot = idle - prev[0], tot - prev[1]
            if d_tot <= 0:
                continue
            pct = 100.0 * (1.0 - d_idle / d_tot)
            if key == "cpu":
                total = pct
            else:
                cores.append((int(key[3:]), pct))
        cores.sort()
        return total, [round(p, 1) for _, p in cores]


def cpu_temp_path() -> str | None:
    """The CPU package temperature sensor, if the platform has one.

    coretemp's temp1_input is "Package id 0" on Intel; acpitz is the fallback
    and is a chassis sensor rather than the die, so it is only used when
    coretemp is absent and it is worth knowing the difference.
    """
    for want in ("coretemp", "k10temp", "acpitz"):
        for node in sorted(glob.glob(os.path.join(HWMON, "hwmon*"))):
            if _read(os.path.join(node, "name")) != want:
                continue
            p = os.path.join(node, "temp1_input")
            if _read_float(p) is not None:
                log.info("CPU temperature from %s (%s)", p, want)
                return p
    UNAVAILABLE["temp_c"] = NOT_EXPOSED
    log.warning("no CPU temperature sensor under %s", HWMON)
    return None


# ---------------------------------------------------------------- PCM power

_PCM_PREFIX_RE = re.compile(r'^\d+\|"[^"]*"\s*\|\s*')


def parse_pcm_energy_joules(output: str) -> float:
    """Package energy in joules from one interval of `pcm -csv`.

    The reference's parser, priority included: Proc + DRAM, else System, else
    CPU. PCM prints prelude and summary lines before the table, so the header is
    found by looking for "Date,Time", and every line carries a `1|"..."|` prefix
    that has to come off first.
    """
    lines = [ln for ln in output.split("\n") if ln.strip()]
    header_idx = next((i for i, ln in enumerate(lines) if "Date,Time" in ln), -1)
    if header_idx < 0 or header_idx + 1 >= len(lines):
        return 0.0
    headers = [h.strip() for h in
               _PCM_PREFIX_RE.sub("", lines[header_idx]).split(",")]
    data = [d.strip() for d in
            _PCM_PREFIX_RE.sub("", lines[header_idx + 1]).split(",")]
    if not headers or not data:
        return 0.0

    system_idx = proc_idx = cpu_idx = dram_idx = -1
    for i, h in enumerate(headers):
        hl = h.lower()
        if "joules" not in hl:
            continue
        if "system energy" in hl:
            system_idx = i
        elif "proc energy" in hl:
            proc_idx = i
        elif "cpu energy" in hl:
            cpu_idx = i
        elif "dram energy" in hl:
            dram_idx = i

    def safe(idx: int) -> float:
        if 0 <= idx < len(data):
            try:
                return float(data[idx])
            except ValueError:
                pass
        return 0.0

    if proc_idx != -1:
        return safe(proc_idx) + (safe(dram_idx) if dram_idx != -1 else 0.0)
    if system_idx != -1:
        return safe(system_idx)
    if cpu_idx != -1:
        return safe(cpu_idx)
    return 0.0


class PowerWorker(threading.Thread):
    """Package watts from Intel PCM, on its own thread behind the PMU lock."""

    daemon = True

    def __init__(self) -> None:
        super().__init__(name="pcm")
        self.watts: float | None = None
        self.t: float = 0.0
        self._warned = False
        if not os.path.isfile(PCM_BIN):
            UNAVAILABLE["pkg_w"] = NOT_EXPOSED
            log.warning("PCM not present at %s: package power unavailable",
                        PCM_BIN)

    def _warn(self, msg: str, reason: str = "") -> None:
        # The reason shown in the panel carries PCM's own words when it gave
        # any, so the browser shows what actually went wrong.
        # Compute the reason first and log THAT: logging the argument meant
        # the interesting case -- reason empty, so PCM's own words are used --
        # printed "reporting as:" and nothing at all.
        why = reason or f"{RUNTIME_BLOCKED} [{msg}]"[:300]
        UNAVAILABLE["pkg_w"] = why
        if not self._warned:
            self._warned = True
            log.warning("%s -- reporting as: %s", msg, why)

    def _sample(self) -> float | None:
        # One second of sampling, so joules over that second ARE watts with no
        # conversion. -nc drops the per-core table nothing here reads and
        # -silent drops the banner; both only make the CSV smaller.
        cmd = [PCM_BIN, "1", "-csv", "-i=1", "-nc", "-silent"]
        try:
            with _PMU_LOCK:
                res = subprocess.run(cmd, capture_output=True, text=True,
                                     stdin=subprocess.DEVNULL, timeout=20)
        except (subprocess.TimeoutExpired, OSError) as exc:
            self._warn(f"PCM failed to run: {exc}")
            return None
        if res.returncode != 0:
            # Surface PCM's OWN diagnostic rather than a generic reason. Its
            # failures name their cause precisely and the causes are different
            # problems: rc=126 is execve refused because the bounding set lacks
            # the file capabilities; "NMI watchdog is enabled" means /proc/sys
            # is not writable so PCM cannot turn it off for the measurement;
            # "unsupported processor" would mean the binary predates this CPU.
            # Collapsing them into one grey message costs an afternoon.
            blob = (res.stdout or "") + (res.stderr or "")
            why = next((ln.strip() for ln in blob.splitlines()
                        if "ERROR" in ln or "denied" in ln), "")
            self._warn(f"PCM exited rc={res.returncode}"
                       + (f": {why}" if why else ""))
            return None
        joules = parse_pcm_energy_joules(res.stdout)
        if joules <= 0:
            self._warn("PCM produced no usable energy counters", NOT_EXPOSED)
            return None
        UNAVAILABLE.pop("pkg_w", None)
        return joules

    def run(self) -> None:
        while running:
            t0 = time.monotonic()
            w = self._sample()
            if w is not None:
                self.watts, self.t = w, time.time()
            time.sleep(max(0.2, PMU_PERIOD - (time.monotonic() - t0)))


# ---------------------------------------------------------------- qmassa GPU

def intel_gpu_bdfs() -> list[str]:
    """PCI BDFs of Intel GPU physical functions, in sysfs card order."""
    out = []
    for card in sorted(glob.glob(os.path.join(DRM, "card[0-9]*"))):
        try:
            drv = os.path.basename(os.readlink(card + "/device/driver"))
            if drv not in ("xe", "i915"):
                continue
            if os.path.exists(card + "/device/physfn"):      # a VF, not the PF
                continue
            if not glob.glob(card + "/device/tile*/gt*/engines"):
                continue
            out.append(os.path.basename(os.readlink(card + "/device")))
        except OSError:
            continue
    return out


class GpuWorker(threading.Thread):
    """Busy, frequency and power from qmassa, behind the same PMU lock."""

    daemon = True

    def __init__(self) -> None:
        super().__init__(name="qmassa")
        self.pct: float | None = None
        self.mhz: float | None = None
        self.watts: float | None = None
        self.mem_mb: float | None = None
        self.mem_is_shared = True
        self.t: float = 0.0
        self._warned = False
        self.bdf = (intel_gpu_bdfs() or [None])[0]
        if not os.path.isfile(QMASSA_BIN) or self.bdf is None:
            for f in ("gpu_pct", "gpu_mhz", "gpu_w"):
                UNAVAILABLE[f] = NOT_EXPOSED
            log.warning("qmassa present: %s / Intel GPU found: %s",
                        os.path.isfile(QMASSA_BIN), self.bdf)
        else:
            log.info("qmassa will sample %s", self.bdf)

    def _warn(self, msg: str) -> None:
        for f in ("gpu_pct", "gpu_mhz", "gpu_w"):
            UNAVAILABLE[f] = RUNTIME_BLOCKED
        if not self._warned:
            self._warned = True
            log.warning("%s", msg)

    def _sample(self) -> None:
        ms = max(100, int(PMU_PERIOD * 400))
        fd, path = tempfile.mkstemp(suffix=".json")
        os.close(fd)
        os.unlink(path)
        cmd = [QMASSA_BIN, "-x", "-t", path, "-n", "2", "-m", str(ms),
               "-d", self.bdf, "--drv-options", "xe=engines=pmu"]
        try:
            with _PMU_LOCK:
                res = subprocess.run(cmd, capture_output=True, text=True,
                                     stdin=subprocess.DEVNULL, cwd="/tmp",
                                     timeout=PMU_PERIOD + 20)
            if res.returncode != 0:
                self._warn(f"qmassa rc={res.returncode}: "
                           f"{(res.stderr or '')[-200:]!r}")
                return
            with open(path) as fh:
                recs = [json.loads(ln) for ln in fh if ln.strip()]
        except (subprocess.TimeoutExpired, OSError, ValueError) as exc:
            self._warn(f"qmassa failed: {exc}")
            return
        finally:
            try:
                os.unlink(path)
            except OSError:
                pass

        state = None
        for r in recs:
            if isinstance(r, dict) and r.get("devs_state"):
                state = r["devs_state"][0].get("dev_stats")
        if not state:
            self._warn("qmassa produced no devs_state record")
            return
        # Busy is the max across engines: the GPU is busy when any engine is.
        latest = [v[-1] for v in (state.get("eng_usage") or {}).values() if v]
        freqs = state.get("freqs") or []
        power = state.get("power") or []
        self.pct = round(max(latest), 1) if latest else None
        if freqs and freqs[-1]:
            self.mhz = float(freqs[-1][0].get("act_freq") or 0.0) or None
        if power and power[-1]:
            gw = power[-1].get("gpu_cur_power")
            self.watts = round(float(gw), 2) if gw is not None else None
        # An integrated GPU has no dedicated VRAM: qmassa reports vram_total 0
        # and the real figure is smem_used, system memory the GT is using. Both
        # are carried, with a flag, rather than quietly calling shared memory
        # "VRAM" -- it is the honest reading of the same tile.
        mem = state.get("mem_info") or []
        if mem and mem[-1]:
            vt = float(mem[-1].get("vram_total") or 0.0)
            if vt > 0:
                self.mem_mb = round(float(mem[-1].get("vram_used") or 0) / 1e6, 1)
                self.mem_is_shared = False
            else:
                self.mem_mb = round(float(mem[-1].get("smem_used") or 0) / 1e6, 1)
                self.mem_is_shared = True
        for f in ("gpu_pct", "gpu_mhz", "gpu_w"):
            UNAVAILABLE.pop(f, None)
        self.t = time.time()

    def run(self) -> None:
        if self.bdf is None or not os.path.isfile(QMASSA_BIN):
            return
        while running:
            t0 = time.monotonic()
            self._sample()
            time.sleep(max(0.2, PMU_PERIOD - (time.monotonic() - t0)))


# ---------------------------------------------------------------- NPU

class Npu:
    """Intel NPU: cumulative busy microseconds, frequency, memory.

    No energy attribute exists on this driver, so NPU power is the one figure
    in this file that is genuinely not exposed.
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
            for f in ("npu_pct", "npu_mhz", "npu_mem_mb"):
                UNAVAILABLE[f] = NOT_EXPOSED
            log.warning("no NPU busy counter under %s", ACCEL)
        else:
            log.info("NPU at %s (power: %s)", self.base, NOT_EXPOSED)
        self._prev: tuple[float, float] | None = None

    def sample(self):
        if self.base is None:
            return None, None, None
        busy = _read_float(os.path.join(self.base, "npu_busy_time_us"))
        mhz = _read_float(os.path.join(self.base, "npu_current_frequency_mhz"))
        mem = _read_float(os.path.join(self.base, "npu_memory_utilization"))
        pct = None
        if busy is not None:
            now = time.monotonic()
            if self._prev is not None:
                d, dt = busy - self._prev[0], (now - self._prev[1]) * 1e6
                if dt > 0:
                    pct = round(max(0.0, min(100.0, 100.0 * d / dt)), 1)
            self._prev = (busy, now)
        return pct, mhz, (round(mem / 1e6, 1) if mem is not None else None)


# ---------------------------------------------------------------- main

def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(name)s %(levelname)s %(message)s")
    signal.signal(signal.SIGTERM, _stop)
    signal.signal(signal.SIGINT, _stop)

    pub = Publisher()
    cpu, npu = CpuLoad(), Npu()
    temp_path = cpu_temp_path()
    power, gpu = PowerWorker(), GpuWorker()
    power.start()
    gpu.start()

    sources = {
        "cpu": "/proc/stat",
        "pkg_w": f"{PCM_BIN} 1 -csv (RAPL via MSR; Proc+DRAM > System > CPU)",
        "gpu": f"{QMASSA_BIN} -d {gpu.bdf} (eng_usage max, act_freq, "
               f"gpu_cur_power)",
        "npu": (npu.base or "(absent)") + "/npu_busy_time_us",
        "temp": temp_path or "(absent)",
    }
    log.info("publishing %s every %.1f s; the PMU tools resample every %.1f s",
             topics.PLATFORM, PERIOD, PMU_PERIOD)

    first = True
    while running:
        t0 = time.monotonic()
        cpu_pct, cores = cpu.sample()
        npu_pct, npu_mhz, npu_mem = npu.sample()
        _t = _read_float(temp_path) if temp_path else None
        temp_now = round(_t / 1000.0, 1) if _t is not None else None
        # The first tick has no previous counter to difference against;
        # skipping it costs a second and avoids opening every bar at a flat
        # zero that would look like a reading.
        if not first:
            pub.send(topics.PLATFORM, {
                "cpu_pct": cpu_pct, "cpu_per_core": cores,
                "pkg_w": power.watts,
                "pkg_w_age": round(time.time() - power.t, 1) if power.t else None,
                "gpu_pct": gpu.pct, "gpu_mhz": gpu.mhz, "gpu_w": gpu.watts,
                "gpu_mem_mb": gpu.mem_mb, "gpu_mem_shared": gpu.mem_is_shared,
                "gpu_age": round(time.time() - gpu.t, 1) if gpu.t else None,
                "temp_c": temp_now,
                "npu_pct": npu_pct, "npu_mhz": npu_mhz, "npu_w": None,
                "npu_mem_mb": npu_mem,
                "sources": sources, "unavailable": dict(UNAVAILABLE),
                "stamp": time.time(),
            })
        first = False
        time.sleep(max(0.05, PERIOD - (time.monotonic() - t0)))

    pub.close()


if __name__ == "__main__":
    main()
