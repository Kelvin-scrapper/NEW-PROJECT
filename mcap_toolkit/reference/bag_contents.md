# Bag family: `rosbag2_2026_07_14-12_52_13` (REAL-X excavator, gravel pit)

Reference notes for the MCAP recordings from this platform. Values below are from
`Copy of rosbag2_2026_07_14-12_52_13_0.mcap` (412 MB, 62.4 s, 36 142 messages,
writer `mcap-rust/0.25.0`, all messages CDR-encoded).

The scene: an orange **REAL-X** hydraulic excavator working a gravel pit, other
CAT excavators, people in hi-vis vests. Ego-vehicle returns are already removed
from the lidar (`self_filtered`).

## Topics

| Topic | Type | Msgs | Rate | Notes |
|---|---|---|---|---|
| `/hal/perception/Main/compressed_video` | `foxglove_msgs/msg/CompressedVideo` | 1510 | 24.2 Hz | **H.265**, portrait **1536×1920** |
| `/hal/perception/MastLeftRear/compressed_video` | ″ | 1511 | 24.2 Hz | H.265, **1920×1536** |
| `/hal/perception/MastLeftSide/compressed_video` | ″ | 1535 | 24.6 Hz | H.265, 1920×1536 |
| `/hal/perception/MastRightRear/compressed_video` | ″ | 1514 | 24.3 Hz | H.265, 1920×1536 |
| `/hal/perception/MastRightSide/compressed_video` | ″ | 1525 | 24.5 Hz | H.265, 1920×1536 |
| `/hal/perception/*/camera_info` | `sensor_msgs/msg/CameraInfo` | ~1510 ea | 24 Hz | intrinsics per camera |
| `/livox/lidar_front_left/self_filtered` | `sensor_msgs/msg/PointCloud2` | 618 | 9.9 Hz | frame `livox_front_left` |
| `/livox/lidar_rear_left/self_filtered` | ″ | 613 | 9.8 Hz | frame `livox_rear_left` |
| `/livox/lidar_rear_right/self_filtered` | ″ | 607 | 9.7 Hz | frame `livox_rear_right` |
| `/tf` | `tf2_msgs/msg/TFMessage` | 19 121 | ~307 Hz | dynamic (`map`→`BASE`, `map`→`odom`, cabin slew, arm joints…) |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 1 | — | rigid sensor mounts (below) |

## Camera streams

* Codec **H.265 / HEVC**, Annex-B, one frame per message, **no B-frames**
  (Foxglove `CompressedVideo` forbids them) → message *k* ↔ decoded frame *k*
  once decoding has started.
* Keyframe (IDR + SPS) roughly every **30 frames**. The first message on each
  topic is a P-frame, so a decoder drops the **~1 s of frames before the first
  IDR** (Main: 27 of 1510). Those frames are unrecoverable from this file.
* `pix_fmt` `yuv420p`. Pixel data is correctly oriented as decoded (Main really
  is portrait — a mast camera mounted rotated).
* **Clock quirk:** `CompressedVideo.timestamp` is a *monotonic* clock
  (starts ~407 s), **not** Unix time and **not** comparable to lidar
  `header.stamp`. Use the message **`log_time`** (Unix ns, same era as the lidar
  stamps) for cross-sensor matching.

## Lidar streams (Livox)

* `point_step = 32`, `height = 1`, `is_dense = true`, little-endian.
* Fields: `x,y,z` `float32` @ 0/4/8 · `offset_time` `uint32` @ 16 ·
  `reflectivity` `uint8` @ 20 · `tag` `uint8` @ 21 · `line` `uint8` @ 22.
  (`reflectivity` is the intensity-like field, 0–255.)
* ~11 k–14 k points per message. Non-repetitive scan pattern → a single frame
  looks sparse; density builds up across frames.
* The 3 lidars are near-synchronised: front↔rear-left ~2.5 ms typical,
  front↔rear-right ~11 ms typical (a few 0.5–1.1 s dropouts).
* `header.stamp` **is** Unix time (`1784033536…`).

## `/tf_static` (rigid mounts, ROS quaternion x y z w)

Sensor-relevant subset:

```
CABIN            -> livox_front_left   xyz=(+1.1390,+1.4190,+1.9900) q=(+0.92400,+0.38250,-0.00410,-0.00060)
livox_front_left -> livox_rear_left    xyz=(-2.5690,-2.5420,+0.2620) q=(-0.01380,+0.00350,-0.38740,+0.92180)
livox_front_left -> livox_rear_right   xyz=(-4.5120,-0.5810,+0.2680) q=(+0.00320,+0.01410,+0.92580,+0.37780)
CABIN            -> livox_front_left   (as above)  # camera links hang off the livox_* frames:
livox_front_left -> Main               xyz=(-0.4130,+0.3790,+0.0010) q=(+0.53060,+0.21950,+0.75530,-0.31600)
livox_rear_left  -> MastLeftRear / MastLeftSide
livox_rear_right -> MastRightRear / MastRightSide
```

`BASE` (undercarriage, z = 0 at ground) connects to `CABIN` only through the
**dynamic** `/tf` (the cabin slews), so a static-only pipeline must target
`CABIN` or a `livox_*` frame, not `BASE`.

A ROS transform `(parent, child, R, t)` maps **child-coords → parent-coords**:
`p_parent = R·p_child + t`.

## What was produced for CVAT (2026-09-10)

* `cvat_output/rosbag2_2026_07_14-12_52_13_0_cvat.mp4` — 1920×1920, 25 fps CFR,
  7505 frames (~5:00), H.264. 5 segments: Main `0–1482`, MastLeftRear
  `1483–2965`, MastLeftSide `2966–4478`, MastRightRear `4479–5991`,
  MastRightSide `5992–7504`. Mapping in `frame_segments.csv`.
* `cvat_output/rosbag2_2026_07_14-12_52_13_0_cvat_3d.zip` — 618 fused frames in
  the `CABIN` frame (607 with all 3 lidars, 6 with 2, 5 front-only), mean ~27 k
  pts/frame. Each `related_images/<frame>_pcd/` folder holds the **5 cameras as
  separate images** (`0_Main.jpg` … `4_MastRightRear.jpg`, ≤960 px), so CVAT
  shows one contextual panel per camera. `frame_index.csv` `cam_frames` column
  gives the source frame per camera. Built via
  `mcap_to_cvat_3d.py --img-maxdim 960 --context-topics …` (README example);
  add `--composite` for a single tiled grid image instead.
