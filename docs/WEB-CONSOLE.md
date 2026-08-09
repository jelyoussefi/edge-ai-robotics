# The web console

Watch the demo from any browser on the LAN, because the machine sits away from
the TV. It serves the compositor's **own** annotated frames — it never renders
anything itself, so the browser and the TV cannot disagree about what is on
screen.

```
compositor ──JPEG on COMPOSITED_FRAME──► web ──MJPEG multipart──► browsers
browsers   ──POST /cmd/<action>──► web ──UI_CMD──► compositor
telemetry  ──PLATFORM──► web ──/platform──► browsers
```

---

## 1. LAN only, no authentication

It binds **`0.0.0.0:8080`** — all interfaces, IPv4 and IPv6 — and the compose
port publish carries no host-IP prefix, so Docker exposes it on every interface
too. Verified:

```
$ ss -ltnp | grep 8080
LISTEN 0 4096   0.0.0.0:8080   0.0.0.0:*
LISTEN 0 4096      [::]:8080      [::]:*
$ curl -o /dev/null -w '%{http_code}' http://192.168.1.19:8080/    # 200
```

**Anyone who can reach the port can watch the camera and drive the overlays.**
There is no login, no token and no TLS. That is acceptable on a demo network and
nowhere else. Do not port-forward it, and do not put it on a network you do not
control. The warning is repeated in the module docstring, in the compose block
and on the page itself, so nobody meets it only once.

## 2. Resolution is a calibration-bearing setting

`STREAM_RES` picks the sensor mode: **`720p` (default)** or **`480p`**. One
place decides — `common/edgebot/camera.py` — because three processes have to
agree and they start separately: `source` opens the sensor, `calibrate` reopens
it to read intrinsics, `compositor` sizes its window and scales the intrinsics.

`fx`, `fy`, `ppx`, `ppy` are pixel quantities in a **particular raster**, and
1280x720 is not an enlarged 640x480 — different aspect, different sensor crop.
So changing `STREAM_RES` invalidates both `config/camera_calibration.json` and
`config/floor_mask.png`, and both must be regenerated:

```bash
make calibrate HEIGHT=<metres>
```

Nothing tries to convert one calibration into the other, because nothing can.
Running with a mismatch is the quiet failure this project fears most — the
picture still composites, the overlay still paints something, and every distance
is wrong — so the compositor checks at startup and says so at ERROR level:

```
CALIBRATION IS STALE: it was taken at 640x480 and STREAM_RES=720p runs the
sensor at 1280x720. ... distances, the floor polygon and the obstacle
footprints are all wrong until you run 'make calibrate HEIGHT=<metres>'
```

**What the move to 720p touched.** The intrinsics reference used to be the
literal `640.0` / `480.0` written into about thirty expressions across the
compositor, the calibration tool and the groundfloor bridge. All of them now
scale from the raster recorded in the calibration itself. Two related defects
fell out of doing this:

- `services/groundfloor/bridge.py` read `calib["fx"]`, but the calibration
  writes those under `calib["intrinsics"]`. Every intrinsic was therefore
  falling back to its hard-coded default — which happened to be the 640x480
  D455 values, so it worked by coincidence and would have published a
  `CameraInfo` describing the wrong lens the moment the sensor changed.
- `RGB_HZ` is 25 against a 30 fps sensor, and the throttle can only publish on
  frame boundaries, so it delivers **15 Hz**, not 25. Unchanged here, but the
  backdrop updates at half the composite rate and that is why.

**YOLO is unaffected.** `detector.py` takes its input size from the model's own
port (`640x640`) and letterboxes whatever frame it is given, aspect preserved.
Confirmed at 720p: `yolo11m-seg.xml compiled for NPU, input 640x640`.

### Measured cost

Same room, same scene, consecutive runs. `before` is the configuration as it
shipped; the two `STREAM_RES` columns are what ships now.

| | before: 480p, scale 3, 960x720 out | **720p**, scale 2, 1280x720 out | **480p** fallback, scale 2, 640x480 out |
|---|---|---|---|
| composited fps | 30.1 | 29.9 | 30.1 |
| JPEG encode / frame | 2.3 ms | **3.1 ms** | **1.1 ms** |
| compositor frame work, median / p95 | 30.4 / 35.8 ms | 29.2 / 36.4 ms | 31.3 / 34.3 ms |
| compositor container CPU | — | **65.9 %** | **29.7 %** |
| source container CPU | — | 56.2 % | 22.9 % |
| bus total, per subscriber | 8.72 MB/s | **20.65 MB/s** | **7.74 MB/s** |
| ├ `camera.depth` | 5.95 MB/s @ 614 kB | 15.11 MB/s @ 1843 kB | — |
| ├ `compositor.frame` | 2.07 MB/s @ 69 kB | 3.57 MB/s @ 119 kB | — |
| └ `camera.rgb` | 0.61 MB/s @ 41 kB | 1.87 MB/s @ 117 kB | — |
| web latency, median / p95 | 7 / 11 ms | 7 / 12 ms | 6 / 9 ms |

Reproduce with `make bus-rate ARGS="--seconds 25"`, `make web-latency`, and the
compositor's own 30 s log line.

Reading it:

- **The bus is what 720p actually costs**: 2.4x, and almost all of it is raw
  depth at 1.84 MB per message. That is the number to watch, not the frame rate.
- **`RENDER_SCALE` had to drop to 2.** The offscreen buffer is window x scale in
  both dimensions, so cost grows as the square: at 1280x720 a scale of 3 would
  ask for 3840x2160 while the compositor was already spending 30.4 of its 33 ms
  budget at 480p. At 2 the offscreen is 2560x1440 — *fewer* pixels than the
  2880x2160 it used before — which is how the resolution went up while the
  per-frame work did not.
- **Container CPU is the honest load signal here, not frame work.** The
  per-frame figure barely moves across a 3x change in rendered pixels, so it is
  dominated by a fixed synchronisation cost (the readback and the buffer swap)
  rather than by pixel throughput. Compositor CPU going 29.7 % -> 65.9 % is the
  real price.
- **Latency did not move**, which is the useful result: 720p costs bandwidth and
  CPU, not responsiveness.

**When to fall back to 480p:** a congested LAN, or more than two or three
simultaneous viewers. Each viewer costs its own copy of the composited stream —
3.57 MB/s at 720p, about 1.4 at 480p.

## 3. Latency, and which half of it is measured

The compositor burns `time.time()` into the bottom-left of every frame **and**
sends the same value as the part's `X-Stamp` header.

`make web-latency` reads the header and subtracts it from arrival. That covers
JPEG encode -> bus -> web service -> HTTP socket: **median 7 ms, p95 12 ms at
720p**. It does **not** cover the browser's own decode and paint, which is
exactly why the stamp is also in the pixels — photographing a screen next to a
clock is the only way to include them, and that needs a person.

Against the < 300 ms acceptance, the machine-side half leaves 288 ms of margin.

### The bug this depended on

For a while the stream delivered **one frame per client and then went silent**.
The cause was one argument in `common/edgebot/bus.py`:

```python
msgpack.packb(payload, use_single_float=True)
```

A float32 has a 24-bit mantissa, so around a Unix epoch of 1.79e9 its ULP is
**2^31 / 2^24 = 128 seconds**. Every `time.time()` on every topic was being
snapped to a 128 s grid. The stream only writes frames whose stamp differs from
the last one it sent, and the stamp did not change for two minutes at a time.
The same line explains the frame ages that sawtoothed between -64 s and +64 s,
which had been read as container clock skew and was not.

Removing it costs a handful of bytes per message — payloads here are dominated
by JPEG and by raw byte blobs, and `SUITE_CLOUD` packs its points itself
precisely so they never go through msgpack as floats. The stream went from
1 frame per client to **30.1 fps, 2.17 MB/s**, and frame age from -46 s to
0.01 s.

## 4. Status panel

Deliberately carries **no frame count, frame age or encode time**. Those
describe the console's own plumbing; a viewer wants to know what the robot is
doing. They remain measurable — the compositor logs the encode cost and
`make web-latency` reads the stamp.

What is shown: obstacle source, nav mode, map cells known and occupied, active
goal, planned path length, clearance, robot position, stream resolution.

## 5. Platform panel

CPU, GPU and NPU load and power, from a small `telemetry` service publishing
`platform.telemetry` at 1 Hz. Gauges and 60 s sparklines are hand-written inline
SVG — an arc and a polyline — with no charting library.

Read straight from sysfs. `qmassa` and `intel_gpu_top` read the same attributes;
going through one would add a binary, a parser and another thing that can be
absent, for no extra measurement.

| reading | source | this board |
|---|---|---|
| CPU busy, total and per thread | `/proc/stat` | yes |
| Package / core / uncore / dram / psys power | `/sys/class/powercap/intel-rapl:*/energy_uj` | yes |
| GPU busy | `xe` `tile0/gt0/gtidle/idle_residency_ms` | yes |
| GPU frequency, achieved and requested | `freq0/act_freq`, `freq0/cur_freq` | yes |
| GPU power | any `card*/device/hwmon/*/power1_average` | **absent** |
| NPU busy | `accel0/device/npu_busy_time_us` | yes |
| NPU frequency, memory | `npu_current_frequency_mhz`, `npu_memory_utilization` | yes |
| NPU power | — | **absent** |

Live sample:

```json
{"cpu_pct": 96.4, "pkg_w": 56.16, "core_w": 46.05, "uncore_w": 3.67,
 "dram_w": 1.37, "psys_w": 78.29, "gpu_pct": 37.6, "gpu_mhz": 2500.0,
 "gpu_w": null, "npu_pct": 32.0, "npu_mhz": 950.0, "npu_w": null,
 "npu_mem_mb": 134.1,
 "unavailable": {"gpu_w": "not exposed by this driver",
                 "npu_w": "not exposed by this driver"}}
```

### Missing is rendered as missing

Every figure is a number or `null`, and **`null` never renders as 0**. A gauge
reading 0 W for a running NPU is a lie shaped like a measurement, so the widget
draws an empty ring and prints the reason.

There are two different reasons and the panel prints whichever applies:

- **"not exposed by this driver"** — a fact about the hardware. True here of
  iGPU power (the `xe` driver registers no hwmon node; `/sys/class/hwmon` holds
  `acpi_fan`, `acpitz`, `nvme`, `ucsi`, `asus`, `coretemp`, `iwlwifi` and
  nothing for the GPU) and of NPU power (no energy attribute at all).
- **"blocked by the container runtime"** — a fact about how we launched, and
  fixable. The hwmon path is still probed every start, so a driver update starts
  reporting power with no code change: *not exposed* stays a measurement rather
  than a belief.

`act_freq` reads **0 while the GT is gated at the sampling instant**, which
happens even at 40 % busy because busy is averaged over the second and the
frequency is a spot reading. Both the achieved and the requested frequency are
published rather than one number that would quietly mean two things.

All counters — jiffies, idle residency, busy microseconds — are **differenced**,
so the first tick after start is skipped rather than published as a flat zero.

### What has to be mounted, and the two security options

All read-only:

```yaml
- /sys/class/powercap:/sys/class/powercap:ro
- /sys/devices/virtual/powercap:/sys/devices/virtual/powercap:ro
- /sys/class/drm:/sys/class/drm:ro
- /sys/class/accel:/sys/class/accel:ro
security_opt:
  - systempaths=unconfined
  - apparmor=unconfined
```

`/sys/class/*` holds only symlinks into `/sys/devices`, and mounting
`/sys/devices` wholesale does not work — the bind is not recursive and the
powercap subtree arrives empty — so the real directory is mounted as well.

**Both security options are needed, established by testing each:**

1. Docker masks `/sys/devices/virtual/powercap` with an empty read-only tmpfs by
   default, hardening against the RAPL power side-channel (CVE-2020-8694). The
   bind mount is applied and then covered. Symptom: `No such file or directory`.
2. With the mask gone, the default AppArmor profile still denies the read, even
   though the container is root and the file is `0400 root:root`. Symptom:
   `Permission denied`.

Scoped to the `telemetry` container, which only reads read-only sysfs. It is not
`privileged`. **If you would rather not loosen this, delete both lines** — the
collector then reports `blocked by the container runtime` and the panel says so,
instead of inventing 0 W.

The collector costs **0.1 % CPU and 9.6 MB**.

## 6. Overlay controls

Buttons for `f` floor, `s` detections, `p` suite cloud, `m` map, `r` reset. They
publish `UI_CMD`, which carries an **action, never a state** — so the compositor
stays the single owner of what is displayed, cycling actions advance one step
exactly as a keypress does, and a page opened mid-demo cannot reset anybody's
overlays. The keyboard on the machine keeps working: a demo is often driven from
the machine and watched from a phone.

## 7. X is still required — for rendering, not for viewing

`DISPLAY_MODE` is `web`, `glfw` or `both` (default `both`).

`web` skips **presenting**, not the window. The GLFW window carries the GL
context MuJoCo renders through, and this driver refuses to make a context
current on an unmapped X11 drawable, which would turn every GL call into a
no-op. EGL is not an option here either — it fails with a `gladLoadGL` error, as
recorded in `CLAUDE.md`.

So the X session and `xhost +local:root` remain necessary even when nobody is
looking at the machine. That is a deviation from "web mode has no window" and it
is deliberate.
