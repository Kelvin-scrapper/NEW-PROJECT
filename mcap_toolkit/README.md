# mcap_toolkit — rosbag2 `.mcap` → CVAT

Scripts to turn a rosbag2 **MCAP** recording into data you can upload to
[CVAT](https://www.cvat.ai/):

| Script | What it makes | Target |
|---|---|---|
| `inspect_mcap.py` | prints topics, message types, rates, schemas, `/tf_static` | — |
| `mcap_to_cvat_video.py` | H.264 **MP4** from the camera streams | CVAT 2D (video) |
| `mcap_to_cvat_3d.py` | **ZIP** of fused `.pcd` frames + contextual camera images | CVAT 3D (point cloud) |
| `mcap_to_supervisely_pce.py` | **ZIP** Point Cloud Episodes project (+ photo context, + `--start/--count` slice) | Supervisely 3D |
| `compress_video_for_size.py` | shrink any MP4 under a hard size cap (default 24 MB) by frame-decimating + resizing + auto-picking the best CRF | Supervisely video (25 MB) |

No ROS installation is required — everything runs from `pip` packages, and
`av` (PyAV) bundles its own FFmpeg (so no system `ffmpeg` needed either).

CVAT tasks are **either** 2D **or** 3D, never both — the video and the point
cloud are two separate tasks/uploads.

---

## Setup (once)

```powershell
cd "C:\Users\Execo Training\Desktop\Test project\mcap_toolkit"
py -m pip install -r requirements.txt
```

## Typical run

```powershell
# 1. see what's in the bag
py inspect_mcap.py "..\Copy of rosbag2_2026_07_14-12_52_13_0.mcap"

# 2. cameras -> one combined MP4 (2D task)
py mcap_to_cvat_video.py "..\Copy of rosbag2_2026_07_14-12_52_13_0.mcap"

# 3. lidars -> fused point-cloud ZIP, each frame carrying all 5 cameras as
#    separate CVAT context panels (3D task)
py mcap_to_cvat_3d.py "..\Copy of rosbag2_2026_07_14-12_52_13_0.mcap" ^
    --img-maxdim 960 --context-topics ^
      /hal/perception/Main/compressed_video ^
      /hal/perception/MastLeftSide/compressed_video ^
      /hal/perception/MastRightSide/compressed_video ^
      /hal/perception/MastLeftRear/compressed_video ^
      /hal/perception/MastRightRear/compressed_video
#    add --composite to get one tiled grid image instead of 5 panels

# 4. (alternative target) lidars -> Supervisely Point Cloud Episodes zip,
#    first 40 frames only, as a quick test upload
py mcap_to_supervisely_pce.py "..\Copy of rosbag2_2026_07_14-12_52_13_0.mcap" ^
    --count 40 --img-maxdim 640 --context-topics ^
      /hal/perception/Main/compressed_video ^
      /hal/perception/MastLeftSide/compressed_video ^
      /hal/perception/MastRightSide/compressed_video ^
      /hal/perception/MastLeftRear/compressed_video ^
      /hal/perception/MastRightRear/compressed_video
```

> **Git Bash note:** topic paths starting with `/` get mangled into Windows
> paths by MSYS. Prefix the command with `MSYS2_ARG_CONV_EXCL='*'` or just run it
> from PowerShell / cmd.

Outputs:

```
cvat_output\<stem>_cvat.mp4          combined camera video (5 segments back-to-back)
cvat_output\frame_segments.csv       which video frame range = which camera
cvat_output\<stem>_cvat_3d.zip       pointcloud/NNNNNN.pcd + related_images/NNNNNN_pcd/<n>_<cam>.jpg
supervisely_output\<project>.zip     Supervisely Point Cloud Episodes project
```

Then follow **`reference/cvat_upload.md`** or **`reference/supervisely_upload.md`**.

---

## `inspect_mcap.py`

```
py inspect_mcap.py BAG.mcap
py inspect_mcap.py BAG.mcap --first-message /livox/lidar_front_left/self_filtered
py inspect_mcap.py BAG.mcap --tf
py inspect_mcap.py BAG.mcap --schema CompressedVideo
```

## `mcap_to_cvat_video.py`

Auto-detects `foxglove_msgs/msg/CompressedVideo`, `sensor_msgs/msg/CompressedImage`
and `sensor_msgs/msg/Image` topics.

| flag | default | meaning |
|---|---|---|
| `--out PATH` | `cvat_output/<stem>_cvat.mp4` | output file (or dir with `--per-camera`) |
| `--topics …` | auto-detect | explicit camera topics **and their order** |
| `--per-camera` | off | one MP4 per topic instead of one combined |
| `--canvas WxH` | square of the largest dimension | combined-canvas size |
| `--fps N` | `25` | constant output frame rate |
| `--crf N` | `21` | x264 quality (lower = better/bigger) |
| `--preset P` | `faster` | x264 speed/size preset |
| `--no-label` | off | don't stamp the camera name/resolution |

Every camera's native pixels are centre-placed on the canvas with black padding
(no scaling, no distortion). The yellow `Name WxH` label is drawn on the padding
for landscape cameras. Output is constant-frame-rate so CVAT frame numbers are
stable.

## `mcap_to_cvat_3d.py`

Auto-detects `sensor_msgs/msg/PointCloud2` topics and fuses them.

| flag | default | meaning |
|---|---|---|
| `--out PATH` | `cvat_output/<stem>_cvat_3d.zip` | output zip |
| `--lidar-topics …` | auto-detect | explicit lidar topics |
| `--ref TOPIC` | topic with most messages | reference clock; defines output frames |
| `--target-frame F` | `CABIN` | frame the merged cloud is expressed in (resolved via `/tf_static`) |
| `--match-tol S` | `0.06` | max timestamp gap to fold another lidar into a frame |
| `--context-topic T` | none | **one** camera topic → one contextual image per frame |
| `--context-topics …` | none | **several** camera topics → one image **each** per frame (separate CVAT panels) |
| `--composite` | off | with `--context-topics`, tile them into one grid image instead of separate panels |
| `--context-cols N` | `3` | composite grid width (with `--composite`) |
| `--cell WxH` | `640x480` | composite cell size (with `--composite`) |
| `--no-context` | — | skip contextual images |
| `--img-maxdim N` | `1280` | longest side of each per-camera JPEG (use `~960` for 5 cameras) |
| `--jpeg-quality N` | `85` | contextual JPEG quality |
| `--keep-stage` | off | keep the unzipped staging folder |

CVAT renders **every** image in `related_images/<frame>_pcd/` as its own panel, so
`--context-topics a b c` writes `0_a.jpg 1_b.jpg 2_c.jpg` per frame (CLI order =
panel order). `--composite` instead merges them into one labelled grid.

## `mcap_to_supervisely_pce.py`

Same lidar fusion as `mcap_to_cvat_3d.py`, packaged as a **Supervisely Point
Cloud Episodes** project zip (`meta.json` + `key_id_map.json` + episode with
`annotation.json`, `frame_pointcloud_map.json`, `pointcloud/`, `related_images/`).

| flag | default | meaning |
|---|---|---|
| `--start N` / `--count N` | `0` / all | export only a slice of reference frames (test upload) |
| `--project` / `--episode` | `<stem>_pce` / `episode_1` | folder names |
| `--context-topics …` | none | camera topics → one photo-context image + `.json` sidecar per camera per frame |
| `--no-context` | — | clouds only |
| `--img-maxdim N` / `--jpeg-quality N` | `640` / `80` | photo-context image size |
| `--classes a,b,c` | `person,excavator,wheel_loader,truck,car,machine` | `cuboid_3d` classes written to `meta.json` |
| `--target-frame` / `--ref` / `--match-tol` / `--lidar-topics` | as `mcap_to_cvat_3d.py` | fusion controls |
| `--no-calib` | off | skip camera calibration in the sidecars (reference-only images, no cuboid→image projection) |

**Camera calibration is on by default**: each context camera's `*/camera_info`
gives the intrinsics and its `/tf_static` path to `--target-frame` (inverted)
gives the extrinsics, written into `meta.sensorsData` in every `.jpg.json`
sidecar — so a cuboid drawn in the 3D view projects onto each camera panel.
Verified by reprojecting a built cloud back onto its own image (points track
the ground and stop exactly at object edges).

Full upload steps and the `.jpg.json` sidecar / calibration details:
`reference/supervisely_upload.md`.

## `compress_video_for_size.py`

Squeezes an MP4 under a hard size ceiling while keeping every camera segment.
It keeps every Nth frame, resizes, then **searches for the lowest CRF (best
quality) that still fits** — typically 2–3 encodes.

```powershell
py compress_video_for_size.py IN_cvat.mp4 --out OUT_supervisely.mp4 --max-mb 24 --stride 10 --size 1024x1024
```

| flag | default | meaning |
|---|---|---|
| `--max-mb N` | `24` | ceiling in decimal MB; 24 stays under both a 25 MB and a 25 MiB cap |
| `--stride N` | `10` | keep every Nth source frame (25 fps → 2.5 fps) |
| `--size WxH` | `1024x1024` | output size |
| `--preset` | `slower` | x264 preset |
| `--crf-min/--crf-max/--crf-start` | `16/46/30` | search bounds |

If `frame_segments.csv` sits next to the input, an `<out>_segments.csv` maps each
camera to its new frame range. The tool warns if the fit needs CRF > 38 (soft
picture) — then use a smaller `--size` or a larger `--stride`.
Result for `rosbag2_2026_09_15-23_06_38_0`: 460 MB → **22.5 MB**, 850 frames
(170 per camera), 1024×1024, CRF 31.7.

Fusion uses **only `/tf_static`** (rigid sensor mounts), so it is exact and needs
no dynamic TF. Points are written as `float32 x y z intensity` binary PCD.
`frame_index.csv` records, per frame, the timestamp, which lidars were merged,
the point count, and the matched camera frame.

See `reference/` for the details of this specific bag family, the CVAT upload
steps, and known gotchas.
