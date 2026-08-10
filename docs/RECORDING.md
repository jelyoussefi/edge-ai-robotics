# Working without the camera

Record the sensor once, then run the whole stack off the recording. Nothing
downstream knows the difference: same two topics, same payload keys, same shared
timestamp. It is not a second code path — `source` opens a file instead of a
device and everything after it is untouched.

```bash
make record SECONDS=60 OUT=data/salon.db3      # camera required, once
SOURCE_BAG=/data/salon.db3 make run            # camera not required, after
```

`SOURCE_BAG` empty is the camera, and that is the shipped default.

---

## 1. It is `.db3`, not `.bag`

librealsense moved its recorder to the rosbag2 sqlite3 container. 2.58.3, the
version in these images, refuses anything else — and it refuses **inside**
`pipeline.start()`, after the camera is open and the size estimate has printed:

```
RuntimeError: Output file must have .db3 extension: '/data/salon.bag'
```

`record_bag.py` checks the suffix before it touches the device, so the failure
costs a second rather than a camera session. The facility is the one everybody
calls a bag and the prose keeps calling it that; only the extension moved.

## 2. The raster is the trap

`config/camera_calibration.json` describes **one room at one resolution**. `fx`,
`fy`, `ppx`, `ppy` are pixel quantities in a particular raster, and 1280x720 is
not an enlarged 640x480 — different aspect, different sensor crop, 19.9 %
different focal length on this D455.

A recording made at another raster makes **every distance, the floor polygon and
every obstacle footprint wrong** while the picture still composites and the
overlay still paints something. Nothing downstream can detect it. So:

- `record_bag.py` takes its mode from the same `stream_mode()` the source uses,
  over the same `streams.d455.json`. Not a copy of the numbers — a copy is a
  second thing to keep in step, and two places disagreeing about the raster is
  the exact failure being guarded against.
- It reads the **negotiated** profiles back off the device and compares. A
  mismatch is an ERROR, and the partial file is deleted rather than left on disk
  as a trap.
- On playback, `source` compares the file's raster against `stream_mode()` and
  says so at ERROR level, then publishes the **file's** width and height rather
  than the requested ones, because in playback the file decides and the payload
  has to describe what it actually carries.

Recording at 480p and playing it back against a 720p calibration is therefore
loud, twice, instead of silent.

## 3. Size

**About 8 GB per minute at 720p30.** 1280x720 of BGR8 plus 1280x720 of Z16 is
4.4 MB per frame pair, thirty times a second. The estimate and the free space
are printed before the first frame, and the recording refuses to start if it
will not fit.

Measured on this board: 45 s asked, 46.4 s recorded, 1335 frame pairs at
28.7 fps, **6.18 GB**.

`data/` is gitignored. A recording never reaches the repository.

## 4. What is recorded, and what is not

Raw colour and depth. **No alignment and no filters.** `rs.align`, the spatial,
temporal and hole-filling filters all belong to `source` and it applies them on
playback exactly as it does live. Baking them in would freeze one filter
configuration into every future run and make `DEPTH_FILTERS=0` unmeasurable.

Playback loops (`repeat_playback=True`) and runs at the recorded rate.
`BAG_REAL_TIME=0` reads as fast as the file allows, which is occasionally useful
and never a measurement: the whole stack is paced by frame arrival — the
compositor's fps, `ROI_PERIOD`, `OBSTACLE_STALE` — so a bag read flat out
measures the reader.

## 5. Verified against the camera

Same scene, same knobs, `make lane-probe` on the live camera and then on the
recording, a few minutes apart:

| | camera | recording |
|---|---|---|
| occupied cells | 2624 = 6.56 m2 | 2709 = 6.77 m2 |
| floor cells | 4270 = 10.68 m2 | 4199 = 10.50 m2 |
| `roi` polygon | 9 vertices, 1.62-5.60 m, -2.16-2.22 m | 9 vertices, 1.62-5.52 m, -2.16-2.24 m |
| `raw` polygon | 9 vertices, 1.54-5.83 m, -2.38-2.47 m | 9 vertices, 1.54-5.68 m, -2.38-2.49 m |
| published rectangles | 21 | 24 |

**The right comparison is against the camera's own spread, not against zero.**
Three live runs on this scene gave 2624 / 2656 / 2744 occupied cells and 4035 /
4251 / 4270 floor cells. The recording's 2709 and 4199 both fall inside those
ranges, and the bag-to-camera gap (85 and 71 cells) is smaller than the
camera-to-camera one (120 and 235). The recording is indistinguishable from
another sample of the camera.

Topics and payload, read off the bus during playback:

```
camera.depth      8.4 Hz    keys: depth, h, scale, t, w    1280x720  scale 0.0010
camera.rgb       16.3 Hz    keys: h, jpeg, t, w            1280x720
shared stamp: 101 pairs, max |t_rgb - t_depth| = 39.4 ms
stamp is wall clock: age 0.109 s
```

The timestamp is `time.time()` taken once per pipeline iteration and given to
both channels, so a colour frame and the depth frame published in the same
iteration carry **the same** stamp. The 39.4 ms is one frame period and comes
from the two throttles — `RGB_HZ` 25 against `DEPTH_HZ` 10 — skipping different
frames, which happens identically on the live camera because it is the same code
with no branch on the source.

One thing that is **not** identical: the SDK reported `fx=644.5` live and
`fx=643.6` from the recording, a 0.14 % difference, with `ppx`/`ppy` unchanged.
The recorded value is the one on disk in the calibration. It is well below the
raster failure this document is about, and it is not explained here.

## 6. Record what you will need

A recording captures one arrangement of one room. The corridor experiments in
`docs/ETAPE-2` need the coffee table in the corridor arrangement, and no amount
of replay will produce a scene that was not filmed. Record the arrangements you
intend to work on **while the camera is still there**.
