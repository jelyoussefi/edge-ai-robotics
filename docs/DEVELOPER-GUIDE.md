# Developer guide — adopting Intel Robotics AI Suite bricks

Field notes from putting two components of the Intel Robotics AI Suite —
`pointcloud_groundfloor_segmentation` and `adbscan_ros2` — into a running
project that was not built around ROS 2, on one Panther Lake board, against a
live Intel RealSense D455.

Every claim below carries the number it was measured at, on this room and this
camera. The numbers are examples of magnitude, not constants to copy: what
transfers is the method and the failure modes.

Full measurement records: [`ETAPE-B-RESULTS.md`](ETAPE-B-RESULTS.md) (floor
segmentation) and [`ETAPE-C-RESULTS.md`](ETAPE-C-RESULTS.md) (clustering and the
union), both in French.

---

## 1. Building the bricks from source

### The packaged binary is not installable

The obvious route is the AMR Debian package. It does not work, and not because
of anything you did:

1. `ros-jazzy-pointcloud-groundfloor-segmentation` exists, and is linked against
   **PCL oneAPI**, whose runtimes live in `apt.repos.intel.com`.
2. That repository rotated its versions. Only **2026.1.1** remains, while the
   AMR package bounds its dependency at **`< 2026.0.0`**.
3. The two cannot be satisfied together. **The binary is uninstallable for
   everyone** until Intel rebuilds one side or the other.

Check this before assuming a local mistake: it presents as an ordinary
dependency error, and it is not one.

**Build from source instead.** The suite is public and Apache-2.0 —
`github.com/open-edge-platform/edge-ai-suites`, subtree `robotics-ai-suite` —
so this needs nothing from Intel. The component's `CMakeLists.txt` wants
**Ubuntu's standard PCL**; the oneAPI linkage only appears behind
`FUZZTEST_FUZZING_MODE`. Nothing has to be patched.

### Pin the checkout by SHA, and fetch only the component

```dockerfile
ARG SUITE_REPO=https://github.com/open-edge-platform/edge-ai-suites.git
# Reviewed on 2026-08-06. Bump deliberately, not by rebuilding.
ARG SUITE_COMMIT=d35ad014d42e270630cd7866f38e679b7bd8ea4a

RUN mkdir -p /ws/src && cd /tmp \
    && git init suite && cd suite \
    && git remote add origin ${SUITE_REPO} \
    && git config core.sparseCheckout true \
    && echo "robotics-ai-suite/components/groundfloor/" > .git/info/sparse-checkout \
    && git fetch --depth 1 --filter=blob:none origin ${SUITE_COMMIT} \
    && git checkout FETCH_HEAD \
    && cp -r robotics-ai-suite/components/groundfloor/pointcloud_groundfloor_segmentation \
             /ws/src/ \
    && cd / && rm -rf /tmp/suite
```

Three things earn their place here. **Sparse checkout plus
`--filter=blob:none`** brings **20 MB** instead of the whole monorepo. **Fetching
a SHA rather than a branch** keeps the build reproducible after `main` moves —
GitHub serves reachable SHAs directly, so no full clone is needed. And the pin
is what makes the component a **measurement baseline**: a brick that silently
changed under you would invalidate every comparison in section 4.

`colcon build` then takes about **16 s** for this component.

### One container per brick, on a shared base

Both bricks need the same ~4.6 GB of Ubuntu 24.04, ROS Jazzy `ros-base`, PCL,
Eigen and the colcon toolchain. Building that twice is wasteful; merging the
bricks into one container is worse, because it couples two components that fail
independently.

The answer is a shared **base image**, `docker/ros-base/`, with no `ENTRYPOINT`
— it is shared *layers*, not a service, and is never run. Measured on this
project:

| | before | after |
|---|---|---|
| each brick | ~4.8 GB | 4.812 GB shared + **5.8 MB / 17.9 MB** unique |
| total image store | 42.2 GB | **37.8 GB**, while adding a third image |

Two rules keep it that way:

- **`COPY common /opt/edgebot` stays in the bricks, never in the base**, and
  comes *after* the `colcon build`. In the base, editing one line of
  `common/edgebot/bus.py` would invalidate the base and with it every layer of
  both bricks, including the multi-minute workspace build. Late in the brick it
  costs one thin layer.
- **Nothing version-pinned to a component belongs in the base.** A base that
  changed when a brick's `SUITE_COMMIT` changed would defeat the point.

**Build through compose, never a bare `docker build`.** `docker-compose.yml`
maps each brick's `FROM` to `service:ros-base` via `build.additional_contexts`,
so compose rebuilds a stale base *before* the brick. A bare `docker build` would
silently use whatever image happens to carry the tag.

### Dependency traps

- `nav2-common` is required — the launch file imports `RewrittenYaml` from
  `nav2_common.launch`. **`nav2-bringup` is not**, and must not be added: it
  drags the entire Nav2 stack in for one launch file you do not use.
- `nav2_dynamic_msgs`, which ADBSCAN publishes, has **no Jazzy binary**. Build
  the `msgs` package only, from `navigation2_dynamic`, pinned.
- Jazzy, not Humble. The suite components ship for both, and Jazzy is the Ubuntu
  24.04 pairing — the same distribution the rest of the project uses. The
  version clash feared at the outset does not exist.

Useful defaults to know before first launch:
`realsense_groundfloor_segmentation_launch.py`, arguments `standalone`,
`camera_name`, `with_rviz`; node defaults `base_frame: base_link`,
`max_surface_height: 0.05`, sensor name `camera`.

---

## 2. Integration pitfalls

### QoS: the failure that looks like nothing happening

Their groundfloor node subscribed **RELIABLE** while the bridge published
**BEST_EFFORT**, as a real camera driver would. DDS refuses to match those,
prints *"no messages will be sent"*, and then produces **30 s of silence** that
reads exactly like a topic-name typo. Fixed by passing
`use_best_effort_qos:=True` at launch rather than by weakening the bridge
(`fed42a8`).

The second brick behaves differently, and the difference is worth writing down
because the reflex learned above is wrong for it:

- **ADBSCAN has no such option and needs none.** `adbscan_sub` subscribes with
  `rclcpp::SensorDataQoS()` unconditionally, which is already BEST_EFFORT and
  already matches.
- **Its output goes the other way.** Its `ObstacleArray` publisher is a plain
  `rclcpp::QoS(1)`, i.e. RELIABLE, so a bridge subscribing to it must be
  RELIABLE too.

Check the publisher and the subscriber separately, per topic. "The suite uses
best-effort" is not a property of the suite.

### `set -u` and ROS setup scripts

ROS setup scripts are **not nounset-clean**: `setup.bash` reads
`AMENT_TRACE_SETUP_FILES` before assigning it. Under `set -u`, which any careful
entrypoint sets, the container dies on line one before running any of its own
code. Relax it around the sourcing only:

```bash
set +u
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
if [ -f /ws/install/setup.bash ]; then
    source /ws/install/setup.bash
fi
set -u
```

Two details that each cost a session: the workspace guard is an **`if`, not
`[ -f … ] && source …`**, because a false test as the last command of a sourced
file makes the `source` itself return non-zero, and under `set -e` that turns a
missing workspace into a silent `exit 1` with no message. And **order matters** —
`/opt/ros` first, then the overlay, or the workspace's own packages lose to the
installed ones.

### Rebuilds: what invalidates what

- Editing anything under `common/` invalidates **every** service image.
  `make suite-compare` rebuilds nothing, so after adding a bus topic you must
  rebuild `perception` and `compositor` by hand.
- `docker compose up -d <service>` **restarts the container with the existing
  image**. It does not rebuild. This produced two measurement runs against code
  that no longer existed in the tree before it was noticed; the tell was
  grepping the running container for a comment that should have been there:

  ```bash
  docker compose exec -T sim grep -c "span guard has to survive" /app/navigator.py
  ```

  Use `up -d --build <service>` after any source change you intend to measure.

### The X session

The compositor pins `MUJOCO_GL=glx` — EGL fails here with a `gladLoadGL` error —
so it needs a real X display, and `make run` calls `xhost +local:root`.

The Makefile and compose both default `DISPLAY` to `:0`. **If the host session is
on `:1`, the compositor crash-loops** with `glfw init failed` preceded by
`Authorization required, but no authorization protocol specified`. Find the real
display before blaming the GPU stack:

```bash
who                     # the session's display is in the last column
ls /tmp/.X11-unix       # X0, X1, …
DISPLAY=:1 xhost +local:root
DISPLAY=:1 docker compose up -d compositor
```

### Impossible sensor returns

Depth produces returns that are not physically possible, and one of them is
enough to ruin a footprint. Measured on this camera: **z down to −6.096 m** under
a floor at 0, and **x up to 14.378 m** in a room whose far wall is at 6.2 m —
about **2.6 %** of points. A median ignores them, but a bounding box is a
maximum, so a single point stretches a rectangle across the room.

Discard them before building anything (`GF_Z_MIN`, `GF_X_MAX`). Keep points
**above** the robot deliberately: a shelf or a low ceiling is a real obstacle,
and that judgement belongs to the navigator, not to the reader.

### Frames

Verify the transform by measurement rather than trusting the tree. Here the
ground intersection came out at **+6.22 m** against +6.2 m expected, and the
optical axis at **+14.08°** below horizontal, matching the calibration — which
is what ruled TF out as the cause of a later disagreement.

The suite's own frame convention is a lever, not just a constraint.
`groundfloor` publishes in `base_link`, whose origin is the ground point under
the camera; adopting that as the project's world frame means **cluster positions
come back needing no transform at all**. Getting there required setting ADBSCAN's
`Lidar_type` to `3D` rather than `RS`: the `RS` path rotates every point by
`(x,y,z) <- (z,-x,-y)`, the optical-to-body swap, because it expects a raw
RealSense cloud in the optical frame, and applying it to an already-body-frame
cloud scrambles it.

### Throughput

Their groundfloor node consumes roughly **one depth frame in three** — 277
frames in, 91 segmentations out over 30 s, about **9 Hz against 3 Hz**. Stable
and consistent with BEST_EFFORT and a depth queue of 2. Not a bug, but the suite
sees a third of what the camera produces, and any latency budget has to say so.

---

## 3. Configuring for your sensor, not their robot

Every default in these components was tuned for a particular machine — a
level-mounted camera on an AAEON AMR, or a sparse LiDAR. None of them are wrong;
all of them are somebody else's sensor.

### `max_surface_height` against D455 noise

The single highest-value parameter change in the whole exercise. Their node
accepted as floor only what lay within **0.05 m** of its estimated plane. But
their *own* floor class has a median of **+0.060 m** (σ = 0.094 m over 1.5 M
points): the threshold sat **below the median of the floor it was meant to
accept**, and discarded more than half of it.

Raised to **0.08 m**, the `floor_h_tol_m` of our calibration, so both detectors
tolerate the same deviation from plane:

| | **0.05** (default) | **0.08** |
|---|---|---|
| floor IoU (median) | 0.107 | **0.307** |
| our area | 7.88 m² | 8.08 m² |
| their area | 4.36 m² | **9.33 m²** |
| median boundary | 0.430 m | **0.328 m** |
| p95 boundary | 1.790 m | **1.082 m** |
| floor points / frame | ~25 k | **~61 k** |
| their footprints | 1.8 | 6.1 |

Their area multiplied by **2.14** and ended up *larger* than ours rather than
converging on it. A side effect worth expecting: accepting more floor broke
their merge-everything-into-one-block behaviour, and paired footprints became
measurable for the first time.

Depth error on a RealSense grows roughly as the square of distance, so this
threshold is really a statement about how far you intend to see. Mount the
parameter file (`services/groundfloor/params/`) rather than baking it into the
image — turning a knob must not cost a 4.8 GB rebuild.

### A tilted camera breaks single-threshold ground removal

ADBSCAN removes the ground with **one height threshold** in the frame it is
given. With the camera pitched 14.08° down, the floor **ramps** in a
camera-centred frame:

| world x | z | world x | z |
|---|---|---|---|
| 1.5 m | −1.148 | 4.5 m | −0.418 |
| 3.0 m | −0.783 | 6.5 m | +0.068 |

**1.22 m** of span across the arena. No constant cuts it: the value that clears
the near floor keeps the far floor, and the far floor is a connected sheet —
which came back as a **6.14 × 6.22 m** room-spanning cluster. The fix is not a
better threshold; it is composition (section 5).

### The plane-fit residue, and why the histogram was necessary

After chaining behind `groundfloor` *and* clipping the walls, an arena-sized
cluster still appeared in **60 %** of frames. The per-frame counters could say
"one blob" but not why. Histogramming z of the cloud the clusterer actually
receives found it:

| z band | % of cloud |
|---|---|
| −0.30 to 0.00 | 7.5 % |
| 0.00 to 0.08 | 2.9 % |
| **0.08 to 0.12** | **3.8 %** |
| 0.12 to 0.30 | 10.8 % |
| 0.30 to 1.50 | 62.4 % |

A local peak at 0.10–0.14 m — the highest bin below 0.50 m — sitting just above
the cut. That is the signature of a point-by-point plane fit leaving a scatter
lying *on* the floor rather than making a clean cut.

3.8 % looked too thin to matter, so the second measurement is the one that
justified writing any code. Grid the arena at 10 cm, subsample exactly as the
node does:

| band | pts/cloud | cells | **cells with nothing above 0.30 m** |
|---|---|---|---|
| 0.08–0.12 | 45 | 38 | **71 %** |
| 0.08–0.20 | 111 | 68 | 61 % |
| > 0.30 | 845 | 309 | — |

The last column decides it. Furniture keeps its mass high, so a low return with
**no body above it** is floor the plane fit failed to claim. 38 scattered cells
per cloud is not a carpet — it is enough stepping stones for DBSCAN, which
chains through contact, to cross a room.

Dropping `0 < z < 0.12` before republishing took the match rate from **16 % to
39–42 %** and the arena-wide blob from **60 % to 11–14 %** persistence. Note the
arithmetic: their node already cuts below 0.08 itself (`doDBSCAN.cpp:304`, the
`3D` branch), so the slice actually removed is 0.08–0.12 — **45 of the 1011
points per cloud** that reach the clusterer. **4.5 % of the points were worth 23
points of match rate.**

Keep points *below* the floor. A cluster centred under the floor trips the
bridge's impossible-return filter, and that is the signal that the plane fit has
slipped; cutting them hides it.

### Arena clipping, and doing it upstream

`groundfloor` removes the floor but not the walls. They survive as large
connected sheets, and a chair against a wall stops being a chair — it is one
mass with the wall and half the room.

Clip the cloud **before their node sees it**, not the clusters afterwards: by
then the chair is already inside the merged cluster. Measured against two
unclipped passes from the same session:

| | unclipped | clipped |
|---|---|---|
| clusters in arena | 1.4 / 1.6 | **2.2** |
| matched to ours | 0.8 / 0.9 | **1.1** |
| pair overlap | 0.32 / 0.32 | **0.39** |
| match rate | 27 % / 28 % | **35 %** |
| clustering stage | ~27 ms | **~2.9 ms** |

59 % of points kept. The cost collapse is the interesting part: a 4× penalty
previously blamed on point density turned out to be the **wall sheet**, not the
count.

Make the clip **wider than the region you score on** (here x 1.2–6.3 against a
comparison arena of 1.5–6.5). Clipping exactly at the comparison boundary would
truncate the very clusters being scored, and their extent would become an
artefact of your filter rather than a property of their detector.

### Read the source before tuning a parameter

Two findings that no amount of black-box tuning would have produced:

- **`subsample_LiDAR_data` is a stride, not a fraction.** It keeps one point in
  N (`pIn += ratio`), so *lowering* it increases density. Their 150 is sized for
  a raw RealSense cloud; the input here is what remains after floor removal, a
  small fraction of it. At 75 the clusterer sees twice the points — and finds
  **fewer** objects, not more: the count drops by a fifth, entirely in
  "theirs only", while matched stays flat. That is the signature of a spurious
  cluster being absorbed by a neighbour, not of better detection.
- **`x_filter_back` and `y_filter_*` do not execute on the `3D` path.**
  `doDBSCAN.cpp` applies only the z filter for `dimension == 3`; the x and y
  calls are commented out in their tree, and only the `dimension == 4` (RS)
  branch runs them. Values set there document intent and filter nothing.

---

## 4. Comparison methodology

Most of the effort in this exercise went into measuring the right thing. The
first campaign scored **0.0 matches across 478 comparisons** and the number was
meaningless (`36de12c`).

### Compare like with like

That first campaign compared *our obstacle footprints* against *a floor
segmenter's derived output*. Footprints are a by-product on their side, their
node merges aggressively, and a footprint IoU is diluted the moment either side
merges — even when both agree perfectly on where the objects are. It measured
definitions, not perception.

Two rules came out of it. Compare each component against the thing it is
**for**: a floor segmenter against a floor, a clusterer against obstacle
rectangles. And when a comparison scores zero, suspect the comparison before the
component.

### Separate policy from perception

Our walkable region `roi` is a **policy**: detected floor, minus object
silhouettes, minus obstacle footprints, then shrunk by `ROI_MARGIN` (0.25 m).
Comparing that against their raw segmentation measures our product decisions as
much as their detector. `raw` is **perception alone** — the floor as depth
geometry reports it.

Same 120 s, 694 comparisons, same code, only the definition swapped:

| | `roi` (policy) | **`raw` (neutralised)** |
|---|---|---|
| **median IoU** | 0.315 | **0.530** |
| our area | 8.29 m² | 14.17 m² |
| their area | 9.33 m² | 9.33 m² |
| **median boundary** | 0.328 m | **0.164 m** |
| p95 boundary | 1.025 m | 0.947 m |

IoU up 68 %, boundary halved, **purely by removing our own definitions from the
comparison**. Nothing about either perception pipeline changed. If you publish
one number, publish the neutralised one and say what you neutralised.

### Measure the other side's conventions, do not read them

Their floor class was determined by measurement, not from a config file: over
1.5 M points, class `3` sits at a z median of **+0.060 m** (σ 0.094 m) while
every other class is at 0.77 m or above. That same measurement later exposed a
**+0.060 m plane offset** that is still unexplained — tight enough (σ 0.094 m
over 1.5 M points) to be a systematic fit bias rather than noise, and invisible
to anyone who had trusted the label.

### Rasterise; do not do analytic polygon geometry

Both floor outlines have reflex corners, where polygon boolean operations are
unstable. Rasterise both onto one **5 cm** grid instead — well below the accuracy
either detector actually has — and compute IoU and boundary distance there.

Report boundary distance **symmetrically and as a distribution**, never a mean.
A mean hides the failure that matters: two floors agreeing over most of their
outline while one runs metres past the other along a single wall would still
average small.

When one rectangle must be scored against **several** others, compute the union
exactly by coordinate compression. Summing the members' areas double-counts
their overlaps and depresses the score precisely where a detector fragments an
object into touching pieces — the case you were trying to measure.

### A relaxed metric must be a relaxation

To ask whether a disagreement is only about granularity, we scored our rectangle
against the *union* of their clusters that fall inside it. That comparison is
worthless unless the relaxed matcher can never do worse than the strict one, and
two attempts got it wrong before measurement caught them:

- Assigning purely by coverage let a box join the footprint that covered it most
  while the 1-1 matcher had paired it with a different one — the second
  footprint lost its pair. One trial in 3000 random layouts.
- A coverage-only rule rejects wide boxes that IoU accepts, putting the "relaxed"
  rate at **6 %** against a strict **19 %**.

The fix is structural: **seed the groups with the strict matcher's own pairs**,
then grow them, and let a group whose union scores worse than its best single
member fall back to that member. Then check it — 3000 randomised layouts
asserting grouped ≥ 1-1.

### Per-session validity, and the control variable

**This is the single most important methodological point in the guide.**

Their floor coverage drifts between sessions, on the same room, the same
camera position and unchanged code:

| | étape B | étape C | three later passes |
|---|---|---|---|
| median raw IoU | **0.530** | 0.478 / 0.479 | **0.354 / 0.387 / 0.377** |
| **their area** | **9.33 m²** | ~6.2 m² | **5.95 / 6.35 / 6.42 m²** |
| our raw area | 14.17 m² | 14.17 m² | 14.46 / 14.46 / 14.87 m² |
| median boundary | 0.164 m | 0.294 m | 0.430 / 0.379 / 0.403 m |

The drift is entirely on their side; our geometric floor does not move. It is a
property of a live plane fit, not a regression — but it means **a number from
one session cannot be compared with a number from another**. A match rate of
35 % in one section and 16 % in another are not a regression; they are different
days.

Two practices follow, and everything in sections 3 and 5 was measured under
them:

1. **A/B within one session, always.** Run the before and after back to back,
   ideally in the order A, B, A.
2. **Carry a control variable.** When tuning a parameter that *cannot reach*
   another component — an ADBSCAN setting cannot affect the groundfloor node —
   report that component's output alongside. A stable floor IoU across the passes
   (0.646 / 0.658 / 0.657, or 0.555 / 0.552 / 0.547 / 0.530) proves the scene did
   not move under the measurement. Without it you cannot tell tuning from
   daylight.

And always report **their floor area next to any IoU**. Area is the variable
that moves; IoU is the one that gets quoted.

### Name the disagreement, do not just count it

Per-frame counts say *how many* rectangles neither side matched. They cannot say
whether that is one stable object each side keeps missing or different noise
every frame — and those call for opposite responses. Group the unmatched
rectangles by location and report **persistence**, the fraction of *distinct*
frames in which something appears there.

Leader clustering on the centre at a 0.5 m radius: k-means needs a k you do not
know, and single-link chains along a wall — an unfortunate way to measure a
detector whose defect is chaining along a wall. Count distinct frames, not
rectangles, so two fragments of one object in one frame do not read as double
persistence.

This is what turned "1.4 unmatched clusters per frame" into "one 5.00 × 4.10 m
mass re-cut every frame, plus a real 1.0 × 1.0 m object we never see", and it is
what identified the pillar in section 5.

Ground the physical reading in evidence rather than intuition: the objects here
were named from live YOLO output on the same frame (`dining table` at cx 0.235,
`chair` at cx 0.712 and 0.412, `person` centre) and from the geometry — a 0.40 m
box at 40.6° off axis falls exactly at the edge of the 79° horizontal FOV, where
a dark column stands in the corner of the image.

### State latency honestly

| min | median | p95 | max | mean |
|---|---|---|---|---|
| 28 ms | 41 ms | 49 ms | 52 ms | 39.4 ms |

Median and p95 are under the 50 ms target, **but these are aggregated inputs** —
each value is a figure the bridge prints once per reporting interval, not a
per-frame round trip. The per-frame spread is wider and not observable as
instrumented. The target is therefore **not demonstrated met**, only consistent
with the available measurement. Say that rather than quoting the p95.

---

## 5. The composition result

### Chain the bricks; do not feed each one raw

ADBSCAN on the raw depth cloud returns a 6.14 × 6.22 m room-spanning cluster,
for the tilt reason in section 3. The suite's own answer is composition:
`groundfloor` removes the floor with a tilt-aware plane fit and publishes what
remains, and ADBSCAN clusters points already known not to be floor.

That helped, and it was not sufficient. Out-of-arena rectangles fell from 0.9 to
0.2 per frame — the far-floor sheet was gone — but a **6.51 × 7.88 m** cluster
remained and the match rate was **2 %**. Walls, furniture and ceiling are
themselves one connected mass.

**DBSCAN chains through contact, and there was not one chaining medium but
three.** Each had to be found and removed separately:

| chaining medium | removed by | result |
|---|---|---|
| far floor | chaining behind `groundfloor` | blob survives, match 2 % |
| walls | clipping the cloud to the arena | 27–28 % → **35 %** |
| plane-fit residue | dropping the 0–0.12 m band | 16 % → **39–42 %** |

Final state: **39–42 % match at 0.50 pair overlap, ~3 ms** for the clustering
stage. Expect the same shape of problem with any density-based clusterer on a
dense RGBD cloud: it is not one cause.

### Substitution fails; the union is the architecture

The original plan was substitution — feed the navigator their clusters instead
of ours. The measurement rejects it. Even filtered, they return the right half
of the room as one 3.8 × 2.2 m block in **68–82 %** of frames; unfiltered, a
5 × 4 m rectangle over the whole arena in 60 %. The robot would have nowhere to
walk.

Grouped matching then established that the residue is **not** granularity:
grouping their clusters into ours fires almost never (**1.03 / 1.02** clusters
per group) and moves neither the rate nor the overlap. Their rectangles are not
fragments of ours — they are *larger* than ours. No merge rule closes that.

What the unmatched map shows instead is **complementary blindness**:

| seen by | persistence | size | what it is |
|---|---|---|---|
| us only | 95–100 % | 1.40 × 1.21 m | far kitchen block |
| us only | 41 + 29 % | 2.64 × 0.96 m | the dining table |
| them only | **70–75 %** | 1.0 × 1.0 m | **near pillar and counter** |

We start from semantic segmentation and hold objects *because* they are objects,
even against a wall. They start from a geometric density test and hold the
pillar and counter, which **no COCO class covers**, so we have no footprint there
at all — a real obstacle the robot would walk into.

A false positive costs a detour; a false negative costs a collision. **Union is
therefore the right operator, not the diplomatic one.**

![The robot patrolling in union mode, overlays on](images/etape-c-union-pillar.png)

`DIAG_FRAMES=3 SHOW_FLOOR=1` under `OBSTACLE_SOURCE=union`. Cyan is the suite's
floor contour, red our free floor; the dark column on the right is the pillar.
The overlay draws our footprints only, so ADBSCAN's rectangle is not in the
picture — the detour is evidenced by the log line, not the image.

See also the two floor overlays from the earlier campaign, which make the near-
and far-field disagreements of section 4 legible:
[room occupied](images/etape-b-floors-occupied.png),
[room empty](images/etape-b-floors-empty.png). Both contours go through the
*same* projection, so a visible gap is a difference between detections and not
between two ways of drawing a polygon.

### Implementing a union: three things that are not obvious

**Confirm each source against its own history, then merge.** Detections flicker,
so a footprint must appear in several consecutive updates before the robot acts
on it. With two sources this cannot be pooled: our footprints arrive at ~1 Hz
and their clusters at ~9 Hz, so a shared window lets the fast source fill it and
confirm the slow one's footprints by itself. Worse, an object **only one**
detector ever sees — the pillar is exactly that — would compete against the
other source's updates and never confirm at all.

**Let a stale source fade out.** Each source carries its own timestamp and
contributes nothing past the staleness window, so an optional service that stops
publishing shrinks the union instead of freezing its last obstacles into the
map.

**A size guard must survive the merge, or it does not exist.** Refusing their
3.8 × 2.2 m block cluster by cluster is pointless if three of their sub-3 m
clusters then chain with one of ours inside your own merge step and rebuild it.
Measured: a 5.3 × 3.8 m barrier, **43 escapes and 5 "no way round"** in three
minutes. Apply the guard *during* the merge too, and only when a box from the
untrusted source is involved — our own footprints merge into a 5.3 × 3.6 m
barrier on this scene as well, and that one costs nothing because it lies along
the far edge instead of across the lane.

Four ~215 s passes, same scene:

| | laps | "no way round" | escapes |
|---|---|---|---|
| ours only | 7 / 2 | 0 / 0 | **1 / 3** |
| union, no merge guard | 10 / 4 | **5 / 0** | **43 / 24** |
| union, with guard | 6 / 5 | **0 / 0** | **13 / 12** |

The acceptance test passes: the pillar is detoured repeatedly as a compact
~1.0 × 1.0 m obstacle at (2.0, −1.2), tagged with the source that saw it. Laps
complete at the baseline rate and "no way round" stays at zero.

**The residual cost is real and unresolved:** 12–13 shallow escapes per pass
against 1–3 for the baseline, 0.05 to 0.56 m of penetration. The likely cause is
their pillar rectangle breathing from 0.7 × 0.9 m to 2.6 × 1.0 m within one pass
while the lane runs along its edge — which is also an argument for the
per-source confirmation threshold above being tuned per source, not just
computed per source.

### Guard the new behaviour behind a default-off switch

`OBSTACLE_SOURCE=ours|suite|union`, defaulting to `ours`. The suite bricks are an
optional compose profile, and a navigator whose behaviour depended on whether an
optional service happened to be running would be a bad default in either
direction. Subscribe to the extra topic unconditionally — a silent topic is free
— and make the *mode*, not the subscription, the decision point. Reject an
unrecognised value loudly: silently falling back to the default looks exactly
like the new path quietly doing nothing.

---

## 6. Devices: what each one is actually worth

Everything below was measured on this board — Intel Core Ultra X7 358H, Arc B390
iGPU, AI Boost NPU, OpenVINO 2026.2.0 — during the session that produced this
guide. Where a figure is not instrumented it is marked **unmeasured** rather
than estimated.

### Model placement is a measurement, not a policy

The project splits work CPU / iGPU / NPU by design. Two of the three placements
were checked by timing the same model on all three devices, 500 iterations for
the policy and 40 for the detector, after warm-up, inference only:

**RL locomotion policy** (`walker.onnx`, input `[1, 99]`):

| device | compile | median | p95 | min / max |
|---|---|---|---|---|
| NPU | 134 ms | 0.135 ms | 0.272 ms | 0.112 / 19.855 ms |
| GPU | 1389 ms | 0.135 ms | 0.185 ms | 0.122 / 5.611 ms |
| **CPU** | 86 ms | **0.034 ms** | **0.041 ms** | 0.032 / 0.587 ms |

**The CPU is 4× faster than the NPU here, and the NPU's worst case is 34× its
median.** For a 99-input MLP the dispatch overhead dominates: there is no
arithmetic to amortise it against. At a 200 Hz control loop the budget is 5 ms
and all three devices fit, so this is not a defect — but "the policy runs on the
NPU" is a placement decision that buys nothing measurable, and the tail on a
control path is a reason to look at it.

**Detector** (`yolo11m-seg` FP16, input `[1, 3, 640, 640]`):

| device | compile | median | p95 | min / max |
|---|---|---|---|---|
| NPU | 4.6 s | 13.6 ms | 25.5 ms | 11.6 / 28.2 ms |
| **GPU** | 2.4 s | **9.7 ms** | 18.2 ms | 7.3 / 20.3 ms |
| CPU | 0.5 s | 306.2 ms | 358.1 ms | 281.3 / 367.6 ms |

Here the NPU earns its place — **22× faster than the CPU** — though the iGPU is
faster still at 9.7 ms. The NPU is the right home anyway in this system, because
the iGPU is simultaneously doing the compositing; the point is that the ranking
was measured rather than assumed.

**Inference is not the frame cost.** The running service reports **29–48 ms per
frame, typically 31–33 ms**, against 13.6 ms of NPU inference. More than half
the frame goes to pre- and post-processing — letterboxing, NMS, and assembling
32 mask prototypes at 640×640.

That shows up as CPU load. Three samples, 8 s apart, steady state:

| container | CPU |
|---|---|
| perception | **1356 – 1401 %** (≈ 13.5–14 cores) |
| compositor | 54 – 62 % |
| source | 20 – 22 % |
| sim | 8 – 9 % |

**Offloading a model to the NPU does not make the service cheap.** The split
between numpy post-processing and OpenVINO's own threads is **unmeasured** here;
the total is not.

### NPU driver and Level Zero pinning

Installed and verified in the running containers:

| package | version |
|---|---|
| `intel-driver-compiler-npu` | 1.35.0.20260722-29947505341~ubuntu24.04 |
| `intel-fw-npu` | same |
| `intel-level-zero-npu` | same |
| **`libze1`** | **1.28.2-1~24.04~ppa1** |

The loader is pinned by URL to a PPA snapshot, deliberately **not** taken from
the graphics PPA that supplies the rest:

```dockerfile
# Level Zero loader v1.28.2, the version validated with NPU driver 1.35.0.
RUN wget -q -O /tmp/libze1.deb \
      https://snapshot.ppa.launchpadcontent.net/kobuk-team/intel-graphics/ubuntu/\
20260606T100000Z/pool/main/l/level-zero-loader/libze1_1.28.2-1~24.04~ppa1_amd64.deb
```

**The failure mode is what makes this worth pinning.** A mismatched `libze1`
**breaks the NPU while leaving the GPU working**. Nothing crashes and no error
names the loader. What you see instead is OpenVINO reporting the device as
unavailable and falling back — and if the fallback is silent, a model that
appears to be running on the NPU has quietly been running on the CPU all along.
That is not hypothetical: it is exactly what happened in the `sim` container
before the driver was installed there, and the policy had been on the CPU
throughout.

Two defences, both cheap:

1. **Log the device you actually got, not the one you asked for**, and log *why*
   the fallback happened. The useful part of an OpenVINO error is at the end of
   the chain — the first line is only `Exception from core.cpp:117` — so print
   the last few lines plus `core.available_devices`.
2. **Assert the runtimes at build time**, so a broken image fails in CI rather
   than at 3 a.m.:

   ```dockerfile
   RUN ldconfig -p | grep -q libze_loader || { echo "ERROR: libze_loader missing"; exit 1; } \
       && ldconfig -p | grep -q libOpenCL  || { echo "ERROR: libOpenCL missing";  exit 1; }
   ```

**Mount `/dev/accel`, not `/dev/dri`.** The NPU appears as `/dev/accel/accel0`
(char device 261,0, group 992 here). Without it mounted, `OV_DEVICE=NPU`
silently falls back to CPU — the same invisible failure as above. `/dev/dri`
plus `group_add` for the render and video GIDs is what the *GPU* needs, and
mounting only that is an easy way to think the NPU is configured when it is not.

Do not bump these versions casually. The pin encodes a validated pair, and the
symptom of breaking it is a performance regression with no error message.

### GPU compositing cost

The compositor sustains **30.0 fps**, flat, with `depth paired=True`. That is
the camera's rate, not a limit it is fighting: raising the frame cap `MAX_FPS`
from 60 to **240 left the rate at exactly 30.0 fps**, so the GL stage is
comfortably inside the frame budget and the loop is paced by the camera stream.

**The per-frame GPU cost itself is unmeasured.** There is no per-stage
instrumentation in the render path, and `intel_gpu_top` is not installed on this
host, so no busy-fraction is available either. What can be stated is the bound:
under 33 ms per frame including the MuJoCo offscreen render, the composite
shader and the readback, while sharing the iGPU with nothing else — the detector
is on the NPU. Getting a real number means timing the GL stages with query
objects, or installing `intel-gpu-tools`; neither was done.

The CPU side of the same container is measured: **54–62 %** of one core, which
includes the annotation work that has to happen CPU-side.

### MuJoCo and GLFW gotchas

**The depth buffer convention is not fixed — probe it.** On this driver
MuJoCo's offscreen depth comes back **reversed**, background at 0.0 instead of
1.0. Logged at startup:

```
MuJoCo offscreen depth format is 0x8cad, packed with stencil
robot depth background reads 0.0000 -> REVERSED (far = 0) convention
```

Every depth comparison downstream depends on which one you have, so read a
corner pixel — background by construction, so its value *is* the far value — and
branch on it rather than hard-coding either convention.

**Ask the driver what MuJoCo allocated, do not assume.** Blitting depth between
framebuffers is only legal when the formats are **identical**; a mismatch fails
with `GL_INVALID_OPERATION` while copying nothing, which looks like a black
texture and not like an error. Assuming `DEPTH_COMPONENT24` left the depth
texture at zero for the entire life of this project. Assuming
`DEPTH24_STENCIL8` merely moved the error. The format here turns out to be
`0x8cad` (`GL_DEPTH32F_STENCIL8`), packed with stencil — probe it.

**GLX, not EGL.** `MUJOCO_GL=glx` is pinned because EGL fails here with a
`gladLoadGL` error. That makes the compositor X11-only, which is what section 2
is about.

**With GLFW, CPU-side annotation must go through `gpu.present_image()`.** The
overlay and patrol ring are drawn on the CPU copy, so that copy has to be what
reaches the window; `present()` re-runs the shader from the GPU textures instead
and would show an unannotated frame.

**Cap the frame rate explicitly.** vsync may not throttle at all in a container
GL context, and the loop will spin the GPU and CPU at hundreds of fps and slow
the whole machine. `MAX_FPS` here is an explicit sleep, independent of vsync.

### What is still unmeasured

Stated plainly so nobody quotes a number that does not exist:

- **Per-frame GPU cost** of the composite stage. Bounded under 33 ms, not
  instrumented.
- **CPU attribution inside `perception`**: post-processing versus OpenVINO
  threads.
- **Per-frame end-to-end latency** through the ROS bridges. The 28 / 41 / 49 /
  52 ms figures in section 4 are aggregated, not per-frame round trips.
- **Power draw**, per device or total. Never measured on this board.
- **Thermal behaviour** over long runs. The longest measured pass here is 215 s.

## 7. Checklist

Adopting a suite brick, in the order that worked:

1. Check whether the packaged binary is installable at all. Build from source
   if not; the licence (Apache-2.0) allows it and the CMake wants stock PCL.
2. Pin the checkout by SHA, sparse, `--filter=blob:none`.
3. Put shared layers in a base image; keep `COPY common` and version pins out
   of it; build through compose.
4. Bring the node up alone and confirm messages flow **before** measuring
   anything. Check QoS per topic, per direction.
5. Verify the frame by measurement, not by reading the TF tree.
6. Discard impossible returns before building geometry from them.
7. Find the parameters tuned for their sensor. Read their source for the
   semantics — strides that look like fractions, filters that do not execute.
8. Compare each component against what it is *for*, with your policy layers
   removed.
9. A/B within one session, with a control variable the change cannot reach.
10. Name the residual disagreement by location and persistence before tuning
    against it.
11. Expect complementarity, not replacement. Guard the new path behind a
    default-off switch and measure the failure mode it introduces, not only the
    capability it adds.
12. Time your models on every device before believing a placement. Pin the
    accelerator runtimes, log the device you actually got, and mount the right
    character device — the NPU's failure mode is silence, not an error.
