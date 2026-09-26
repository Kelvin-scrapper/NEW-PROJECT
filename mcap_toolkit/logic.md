# MCAP → Supervisely: the data and the logic

This explains **what is inside our recordings**, **what we turn them into for
Supervisely**, and **why each step works the way it does**. For how to run the
tools, see [README.md](README.md). For raw topic dumps of one bag, see
[reference/bag_contents.md](reference/bag_contents.md).

---

## 1. The big picture

```
 rosbag2 .mcap  (one recording, 1–3 min, 0.4–1.2 GB)
 ├─ 3 Livox lidars ──► fused into ONE point cloud per lidar sweep (10 Hz) ──► pointcloud/000123.pcd
 ├─ 5 cameras (H.265) ─► decoded, nearest image picked per sweep, 640 px ──► related_images/000123_pcd/0_main.jpg …
 ├─ */camera_info ─────► lens intrinsics  ─┐
 └─ /tf_static ────────► sensor mounts ────┴─► calibration in every image's .jpg.json sidecar
                                                   │
                                                   ▼
                    <bag>_pce.zip  =  one Supervisely "Point Cloud Episodes" project
                    (annotate 3D cuboids; they project onto all 5 camera views)
```

One recording → one zip → one Supervisely project with one episode. Every
lidar sweep in the recording becomes one frame. Nothing is sampled or shrunk
to fit a size cap.

---

## 2. The data we are handling

### 2.1 The platform

REAL-X autonomous excavator working a gravel pit. Scenes contain the machine
itself, other excavators, wheel loaders, trucks and people in hi-vis. The
recordings are **rosbag2 in MCAP format**, CDR-encoded ROS 2 messages.

### 2.2 Sensors in each bag

| Sensor | Topic (09-15 / 09-16 bags) | Message type | Rate | Notes |
|---|---|---|---|---|
| Lidar, front-left | `/livox/lidar_front_left/self_filtered` | `PointCloud2` | ~10 Hz | **reference lidar** (most messages) |
| Lidar, rear-left | `/livox/lidar_rear_left/self_filtered` | `PointCloud2` | ~10 Hz | |
| Lidar, rear-right | `/livox/lidar_rear_right/self_filtered` | `PointCloud2` | ~10 Hz | |
| Camera, main | `/hal/perception/main/compressed_video` | `foxglove CompressedVideo` | 23–27 Hz | H.265, portrait 1536×1920 |
| Camera, mast left side | `/hal/perception/mast_left_side/compressed_video` | ″ | ″ | H.265, 1920×1536 |
| Camera, mast right side | `/hal/perception/mast_right_side/compressed_video` | ″ | ″ | ″ |
| Camera, mast left rear | `/hal/perception/mast_left_rear/compressed_video` | ″ | ″ | ″ |
| Camera, mast right rear | `/hal/perception/mast_right_rear/compressed_video` | ″ | ″ | ″ |
| Lens calibration | `/hal/perception/<cam>/camera_info` | `CameraInfo` | per frame | intrinsic matrix K |
| Sensor mounts | `/tf_static` | `TFMessage` | once | where every sensor sits on the machine |
| Machine motion | `/tf` | `TFMessage` | ~300 Hz | cabin slew, arm joints (**not used**, see §5) |

The older July bag uses CamelCase camera names (`/hal/perception/Main/…`,
`MastLeftSide`, …). The batch script detects either spelling.

**Lidar points.** Each message has about 11–14 k points. Every point stores
`x y z` (float32, metres, in that lidar's own frame) plus `reflectivity`
(0–255), which we keep as `intensity`. Points hitting the excavator itself
are already removed (`self_filtered`). Livox lidars use a non-repeating scan
pattern, so a single sweep looks sparse. Three lidars fused together give
about 27 k points per frame.

**Camera video.** H.265 with a keyframe about every 30 frames and no
B-frames. The first message of each stream is **not** a keyframe, so the
decoder can't produce anything until the first keyframe arrives. We lose
about the **first 1 s of every camera** (7–8 frames). The build log shows
this as `offset 7`.

### 2.3 Two clocks — the most important quirk

| Field | Clock | Usable for matching? |
|---|---|---|
| lidar `header.stamp` | Unix time | ✅ yes |
| camera `CompressedVideo.timestamp` | **monotonic** (starts at a few hundred seconds) | ❌ no, it's a different clock |
| MCAP `log_time` (every message) | Unix time, when the recorder logged it | ✅ yes |

So cameras are matched to lidar sweeps by **`log_time`**, never by the
camera's own timestamp.

### 2.4 The batch built on 2026-09-26

| Recording | Frames (lidar sweeps) | Zip size |
|---|---|---|
| `rosbag2_2026_09_15-06_51_50_0` | 675 | 551 MB |
| `rosbag2_2026_09_15-06_54_23_0` | 779 | 624 MB |
| `rosbag2_2026_09_15-06_56_51_0` | 1104 | 919 MB |
| `rosbag2_2026_09_15-06_59_27_0` | 661 | 558 MB |
| `rosbag2_2026_09_15-23_06_38_0` | 654 | 477 MB |
| `rosbag2_2026_09_15-23_10_40_0` | 754 | 564 MB |
| `rosbag2_2026_09_16-14_30_28_0` | 1720 (173 s) | 1274 MB |

`2609/… (1).mcap` files were byte-identical copies of two bags above (same
SHA-256), so they were not built again.

---

## 3. Coordinate frames

Every sensor measures in its own frame. `/tf_static` says how the frames
connect:

```
CABIN ─► livox_front_left ─┬─► main (camera)
                           ├─► livox_rear_left  ─► mast_left_side, mast_left_rear
                           └─► livox_rear_right ─► mast_right_side, mast_right_rear
BASE (undercarriage) ─── only via /tf (moves when the cabin slews) ──► CABIN
```

A ROS transform `(parent, child, R, t)` maps child coordinates into parent
coordinates: `p_parent = R · p_child + t`. We chain these (breadth-first search
over the `/tf_static` graph) to get any sensor → `CABIN`.

**Everything is output in the `CABIN` frame.** `CABIN` is the deepest frame
reachable with **static** transforms only. `BASE` would need the moving `/tf`
(see §5).

---

## 4. The build logic, step by step (`mcap_to_supervisely_pce.py`)

### Step 1 — Read the bag summary
Find every `PointCloud2` topic (the lidars) and the camera topics. The lidar
with the most messages (`lidar_front_left`) becomes the **reference**. **One
reference sweep = one output frame**, so a recording's frame count equals the
front-left message count.

### Step 2 — Build the transform graph
Read `/tf_static`, then resolve each lidar's and each camera's frame to
`CABIN`.

### Step 3 — Fuse the lidars (one point cloud per frame)
For each reference sweep at time *t*:
1. Take the front-left points and transform them into `CABIN`.
2. For rear-left and rear-right, take the sweep whose `header.stamp` is
   **nearest to *t***, but **only if it is within 60 ms** (`--match-tol 0.06`).
   A lidar with a dropout at that moment is simply left out of that frame,
   not paired with a stale sweep.
3. Transform each into `CABIN` (`p = R·p + t`), concatenate, and drop any NaN or
   inf points.
4. Write `pointcloud/NNNNNN.pcd`: PCD v0.7, binary, fields `x y z intensity`
   (float32). Each file is about 0.4–0.8 MB.

### Step 4 — Camera images (photo context)
For each of the 5 cameras:
1. Decode the whole H.265 stream once (PyAV). Pair decoded frame *k* with
   message *k + offset*, where the offset is the frames lost before the first
   keyframe (§2.2).
2. Downscale to **640 px on the long side** (portrait main camera → 512×640,
   mast cameras → 640×512) and save as JPEG quality 80.

For each output frame, pick each camera's image whose **`log_time` is nearest**
to the lidar sweep time. Save it as
`related_images/NNNNNN_pcd/<k>_<camera>.jpg`, with `k` = 0 main,
1 mast_left_side, 2 mast_right_side, 3 mast_left_rear, 4 mast_right_rear. That
order is fixed, so image `2_…` is always the same camera in every frame.

Cameras run at about 25 Hz and lidar at 10 Hz, so the nearest image is normally
within about 20 ms of the sweep.

### Step 5 — Calibration (what makes cuboids project onto the images)
Each image gets a sidecar `<image>.jpg.json`:

```json
{ "name": "0_main.jpg",
  "meta": { "deviceId": "main",
            "sensorsData": { "intrinsicMatrix": [fx,0,cx, 0,fy,cy, 0,0,1],
                             "extrinsicMatrix": [r11,r12,r13,tx, r21,r22,r23,ty, r31,r32,r33,tz] } } }
```

* **Intrinsics** come from the camera's `camera_info.K`. It describes the
  full-resolution image, but we saved a 640 px image, so K is scaled:
  `fx·sx, cx·sx, fy·sy, cy·sy` with `sx = 640-px width / native width`, and the
  same for `sy`. Without that scaling, projections would land in the wrong place.
* **Extrinsics.** Supervisely wants **world → camera**, where "world" is our
  point-cloud frame `CABIN`. `/tf_static` gives camera → CABIN `(R, t)`, so
  we store its inverse `[Rᵀ | −Rᵀt]` as a row-major 3×4 matrix.

With both matrices, Supervisely projects any 3D cuboid into each image:
`pixel = K · [R|t] · point`. This is the same mechanism its nuScenes demo uses
(the screenshot reference). It was verified by projecting a built point cloud
onto its own camera image: the points line up with ground ridges and object
outlines.

### Step 6 — Project files
* `meta.json`: project type `point_cloud_episodes` plus our cuboid classes
  `person, excavator, wheel_loader, truck, car, machine` (change them with
  `--classes`).
* `key_id_map.json`: empty (no annotations yet).
* `<episode>/annotation.json`: empty episode with `framesCount` set.
* `<episode>/frame_pointcloud_map.json`: `{"0": "000000.pcd", …}` for frame order.

### Step 7 — Zip
Everything is built in a `_stage_…` folder, then zipped **uncompressed**
(JPEG and float point data don't compress usefully, and storing is much faster),
then the stage folder is deleted.

```
<bag>_pce.zip
└─ <bag>_pce/                      ← project
   ├─ meta.json
   ├─ key_id_map.json
   └─ <bag>/                       ← episode (named after the recording)
      ├─ annotation.json
      ├─ frame_pointcloud_map.json
      ├─ pointcloud/000000.pcd …
      └─ related_images/000000_pcd/0_main.jpg (+ .jpg.json) … 4_mast_right_rear.jpg
```

---

## 5. Known limitations (read before annotating)

| Limitation | Effect | Why it's accepted |
|---|---|---|
| **Cloud is in the CABIN frame; cabin slew is not removed** | When the upper body swings, the whole scene appears to rotate between frames. A parked truck "moves". | Undoing it needs the dynamic `/tf` (`BASE`↔`CABIN`) per sweep. That's doable, but not built yet. Objects stay correct *within* each frame, and calibration stays exact because cameras and lidars ride on the cabin together. |
| **First ~1 s of camera video is lost** (before first keyframe) | The first ~10 lidar frames show the first decodable image, up to ~1 s later than the sweep. Cuboid projections on those frames can look offset. | The data is not in the file. Only the start of each recording is affected. |
| Camera match has no tolerance | If a camera stream had a gap mid-recording, the nearest image could be older than usual. | No such gaps seen in these bags. Lidar matching does use a tolerance. |
| No per-point motion correction | Livox points carry `offset_time` within the 100 ms sweep, which we ignore. Fast slewing smears a sweep slightly. | Small at excavator speeds. |
| Only the first `/tf_static` message is read | If a later message changed a mount, it would be ignored. | Checked on the 09-16 bag, which has 50 `/tf_static` messages. The 26 sensor-mount transforms are in the first message and never change. Later messages only add `map → attack_point / dump_point / scoop_target / excavation_area / target_terrain / world` (dig-task targets), which we don't use. |
| Lidar dropouts | A few frames have 1–2 lidars instead of 3 (fewer points). | Better than pairing with a stale sweep. |

---

## 6. Decisions and why

| Decision | Reason |
|---|---|
| **Full recordings, no 25 MB cap** | We want all the data in Supervisely. The old 25 MB video and 40-frame slices were only for testing. |
| **One zip per recording** | One project/episode per recording keeps frame order and makes tracking IDs meaningful. The episode is named after the bag, so several can be merged into one Supervisely project without clashing. |
| **640 px images** | About 0.5–1 GB per recording at 640 px, versus about 2–3 GB native. Plenty for recognising objects, and calibration is rescaled so projection stays exact. `--img-maxdim` changes it. |
| **Dated batch folder** `supervisely_batch_<YYYY-MM-DD>/<bag>/<bag>_pce.zip` | Same layout as `cvat_batch_…`. The date says when it was built, and names stay tied to the source recording. |
| **Skip byte-identical copies** | `x (1).mcap` from a re-download gives an identical zip, which wastes build time and upload quota. Detected by SHA-256, not by name. |
| **Supervisely plan limit** is per point cloud: Free 25 MB · Pro 100 MB · Pro+Max 500 MB | Our clouds are under 1 MB, so any plan works. Total account storage is separate, so check it before uploading about 4 GB. |

---

## 7. The scripts

| Script | Use it for |
|---|---|
| `supervisely_batch.py` | **The normal way.** Point it at a folder of bags. It de-duplicates, picks cameras, builds every recording into `supervisely_batch_<today>/`, verifies each, and writes `batch_manifest.csv`. It skips zips that already exist and pass the check (so a re-run resumes). |
| `mcap_to_supervisely_pce.py` | Builds one bag. Use it for test slices (`--start/--count`), different classes, or a different target frame. |
| `verify_supervisely_zip.py` | Checks zips before upload: CRC integrity, naming, frame counts, images per frame, calibration on every image. |
| `inspect_mcap.py` | Look inside an unfamiliar bag first (topics, rates, `/tf_static`). |

```powershell
cd mcap_toolkit
py supervisely_batch.py ..\2609 --dry-run                     # show the plan: builds / duplicates / already done
py supervisely_batch.py ..\2609                               # build everything → ..\supervisely_batch_<today>\
py verify_supervisely_zip.py ..\supervisely_batch_2026-09-26  # re-check a whole batch any time
```

In Git Bash, use `python` instead of `py`. `supervisely_batch.py` passes
Windows paths to the builder itself, so there's no need for
`MSYS2_ARG_CONV_EXCL`.

**Before running on a new kind of bag:** run `inspect_mcap.py` and confirm that
it has the `PointCloud2` lidars, `*/compressed_video` cameras with `camera_info`
siblings, and `/tf_static`. The build log line
`calibration: 5/5 cameras (missing …: none)` confirms calibration was found.
