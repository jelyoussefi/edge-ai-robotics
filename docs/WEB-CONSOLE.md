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

## 4. Layout

Three columns inside one viewport-height shell: **Status left, video centre,
Platform right**. No panel titles — the rows label themselves, and a heading
saying "Status" above rows that obviously are status is a line of pixels the
video could have had.

**It must fit at any zoom, with no scrollbars.** Browser zoom shrinks the CSS
viewport, so a layout that fits at 100 % and overflows at 150 % was measured in
pixels somewhere. Nothing here is:

- the shell is `100%` height with `overflow:hidden`, a flex column;
- `main` is `grid-template-columns: minmax(0,1fr) minmax(0,2.9fr) minmax(0,1fr)`
  — `minmax(0,Nfr)` and not `Nfr`, so a column may be **narrower than its
  content** instead of pushing the grid past the viewport, which is exactly what
  creates a scrollbar at high zoom;
- every column and panel carries `min-height:0`, without which a grid or flex
  child refuses to shrink below its content;
- text is `clamp(9px, 1.05vmin, 15px)` and all spacing is in `em`, so the panels
  scale with the viewport rather than fighting it;
- the video is `width:100%; height:100%; object-fit:contain` — not `max-width`
  with `auto`, which refuses to scale a 1280x720 composite **up** and left it
  sitting at native size in the middle of an empty column on a large screen.

Verified in headless Chrome at three zoom levels, emulated as the CSS viewport a
1920x1080 screen presents at each:

| zoom | CSS viewport | scrollbars | video | status | platform |
|---|---|---|---|---|---|
| 50 % | 3840x2160 | none | 2244x2060, fully in view | fully in view | fully in view |
| 100 % | 1920x1080 | none | 1115x1004, fully in view | fully in view | fully in view |
| 150 % | 1280x720 | none | 741x659, fully in view | fully in view | fully in view |

"Fully in view" is checked against the viewport rectangle, not eyeballed, and
`scrollWidth`/`scrollHeight` are compared with `clientWidth`/`clientHeight` in
both axes. The check is `scripts/zoom_check.js`; run it against a running
console with any Chrome that has puppeteer:

```bash
docker run --rm --user root -e PUPPETEER_CACHE_DIR=/home/pptruser/.cache/puppeteer \
  --network edge-ai-robotics_default -v "$PWD/scripts:/work" -w /work \
  ghcr.io/puppeteer/puppeteer:latest node /work/zoom_check.js http://web:8080/
```

![console at 100 %](images/web-console-v3-100.png)
![console at 150 %](images/web-console-v3-150.png)

## 5. Status panel

Obstacle source, nav mode, map cells known and occupied, active goal, path
length, clearance, stream resolution.

Deliberately absent: frame counts, frame age, encode time — those describe the
console's own plumbing — and the **robot pose row**, which was the raw
`(x, y)` of the base and told a viewer nothing the picture does not.

## 6. Platform panel

Value + bar per row driven by `data-max`, exactly the shape of
`reference/intel-toolkit/metrics-panel.js::setMetricRow`, plus a 60 s sparkline
per engine drawn as hand-written inline SVG. No charting library: an area, a
polyline and a caption are the whole requirement.

`data-max` values are **display ceilings, not measurements**: percentages cap at
100 by definition, and the wattage ceilings (80 W package, 30 W iGPU) are what
this board actually draws rounded up, so a full bar means "working hard" and not
"at a limit the silicon knows about".

### Power: Intel PCM over the MSRs

By the reference's `get_power_usage`: run `pcm 1 -csv -i=1 -nc -silent` for one
interval, find the header row containing `Date,Time`, strip the `1|"..."|`
prefix every line carries, and read the energy columns. Joules measured over one
second **are** watts, so there is no conversion. Counter priority is the
reference's: **Proc + DRAM > System > CPU**. On this board PCM offers
`Proc Energy (Joules)`, `Power Plane 0/1` and `SYSTEM Energy` but no DRAM
column, so Proc is used alone.

**The packaged `pcm` cannot measure this machine.** Debian's 202502-1 exits
rc=1 before touching an MSR:

```
Error: unsupported processor ... CPU family 6 model number 204
Brand: "Intel(R) Core(TM) Ultra X7 358H"
```

Upstream has since added Panther Lake as `PTL = PCM_CPU_FAMILY_MODEL(6, 204)`,
so the image builds PCM from a pinned commit. That is the whole reason for a
source build, and no amount of capability granting substitutes for it.

### GPU: qmassa

`qmassa -x -t <json> -n 2 -m <ms> -d <bdf> --drv-options xe=engines=pmu`, then
the max across `eng_usage` engines — the GPU is busy when any engine is, which
is what xpu-smi and intel_gpu_top mean — plus `act_freq` and `gpu_cur_power`.
Not packaged anywhere (not apt, not crates.io), so it is built from a pinned
upstream commit too.

**Correction to an earlier version of this document:** it stated that iGPU power
is "not exposed by this driver". That was wrong. It is absent from
`/sys/class/hwmon`, which is where the previous collector looked, but qmassa
reports it — measured at **7.9 W** while the demo runs. The only figure on this
board that is genuinely not exposed is **NPU power**: the accel driver publishes
busy time, frequency and memory, and no energy counter.

### The PMU lock

PCM and qmassa both claim hardware PMU counters while initialising, and the
kernel allows one client at a time; whichever starts second fails. They run on
**two separate threads** here — neither is fast enough to sit inside a 1 Hz
loop, PCM taking about 1.3 s and qmassa about 2 s — so a single
`threading.Lock` around the subprocess call is the only thing keeping them
apart. It is not a formality and it is not defensive: remove it and one of the
two starts failing. The lock is released before parsing.

Cheap readings (CPU jiffies, NPU sysfs) stay on the main loop at 1 Hz. Power and
GPU carry whatever the workers last produced, so both ship their own age
(`pkg_w_age`, `gpu_age`) rather than pretending to be instantaneous.

### What the host and the container need

**On the host:** the `msr` kernel module.

```bash
sudo modprobe msr          # and /etc/modules-load.d/msr.conf to persist it
ls /dev/cpu/0/msr          # must exist
```

Without it PCM cannot open an MSR handle and reports so.

**In the image:** the PCM binary carries file capabilities, set at build time as
the reference does.

```dockerfile
RUN setcap cap_sys_rawio,cap_sys_admin,cap_dac_override+ep /usr/local/sbin/pcm
```

**In compose**, established by removing each one and re-testing:

| setting | needed by | symptom without it |
|---|---|---|
| `cap_add: SYS_RAWIO, SYS_ADMIN, DAC_OVERRIDE` | pcm | rc=126, `Operation not permitted` on execve |
| `device_cgroup_rules: 'c 202:* rmw'` | pcm | `EPERM` opening `/dev/cpu/N/msr`, though the node is mounted and visible |
| `security_opt: apparmor=unconfined` | pcm | rc=1 |
| `/dev/cpu:/dev/cpu:rw` | pcm | no MSR nodes |
| `/dev/dri` + `/sys/class/drm:ro` | qmassa | no GPU found |

**qmassa needs none of the capabilities or security options** — measured. And
`systempaths=unconfined`, which the previous RAPL-sysfs collector required to
get past Docker's powercap mask, is gone: power no longer comes from
`/sys/class/powercap`.

A file capability can only grant what the container's **bounding** set already
contains, which is why `setcap` alone is not enough and `cap_add` alone would
be. Both are kept: the setcap so the binary carries its own requirement, the
cap_add so the bounding set permits it.

This is scoped to the `telemetry` container, which is why telemetry is **not**
part of the `web` service. The console is LAN-facing and unauthenticated;
giving it `SYS_RAWIO` and MSR access to save a container would be trading the
one thing worth protecting for nothing.

### Missing is rendered as missing

Every figure is a number or `null`, and `null` never renders as 0 — the row
shows an em dash and a zero-width bar. Two reasons are kept apart because they
are different problems:

- **"not exposed by this driver"** — a fact about the hardware. On this board:
  NPU power, and nothing else.
- **"blocked by the container runtime"** — a fact about how we launched, and
  fixable in compose. This is what appears if the caps, the cgroup rule or the
  AppArmor option are removed.

### Measured

Live during the demo at 720p: CPU 91 %, package **55.4 W**, GPU 23.6 % at
2500 MHz drawing **7.3 W**, NPU 34.9 % at 950-1050 MHz, 134 MB.

The panel tracks real change. Stopping and restarting `perception` — the
service that holds ~1400 % CPU and owns the NPU — gives a clean step:

| | CPU % | package W | GPU % | GPU W | NPU % |
|---|---|---|---|---|---|
| demo running | 91.4 | 55.4 | 23.6 | 7.3 | 34.9 |
| perception stopped | **17.2** | **25.4** | 24.4 | 6.4 | **1.4** |
| restarted | 90.9 | 56.0 | 24.0 | 7.5 | 34.8 |

GPU busy barely moves across that step, correctly: the compositor keeps
rendering whether or not anything is being detected.

**Starting the three suite bricks moves the panel very little** — CPU 91.4 to
90.7, package 55.4 W to 54.4 W. That is not the panel being inert, it is the
board being saturated: `perception` alone already holds the CPU at ~91 %, so
extra work is redistributed rather than added. Worth knowing before reading a
flat gauge as a broken one.

The collector costs **0.1 % CPU and 9.6 MB**, PMU tools included.

## 7. Overlays: keyboard only

The on-page overlay panel is gone. `f` floor, `s` detections, `p` suite cloud,
`m` map and `r` reset still work **at the machine**, unchanged.

The `POST /cmd/<action>` endpoint is deliberately kept even though no button
calls it: it is how a scripted capture drives the overlays without a keyboard,
which is how the figures in `docs/images/` are made. It publishes `UI_CMD`,
which carries an **action, never a state**, so the compositor remains the single
owner of what is displayed.

## 8. X is still required — for rendering, not for viewing

`DISPLAY_MODE` is `web`, `glfw` or `both` (default `both`).

`web` skips **presenting**, not the window. The GLFW window carries the GL
context MuJoCo renders through, and this driver refuses to make a context
current on an unmapped X11 drawable, which would turn every GL call into a
no-op. EGL is not an option here either — it fails with a `gladLoadGL` error, as
recorded in `CLAUDE.md`.

So the X session and `xhost +local:root` remain necessary even when nobody is
looking at the machine. That is a deviation from "web mode has no window" and it
is deliberate.
