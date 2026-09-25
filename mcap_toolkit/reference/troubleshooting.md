# Troubleshooting & gotchas

### "This file isn't playable / unsupported / corrupt" right after conversion
The MP4 writes its seek index (`moov` atom) only when the encoder closes the
container. A run still in progress is a headerless partial file and **no player
can open it**. Wait for `WROTE …` / `ALL DONE` in the log.

### First ~1 s of each camera segment is missing
The H.265 streams start on a P-frame; a decoder cannot produce output until the
first keyframe (IDR), ~30 frames / ~1 s in. Those frames are not in the file in a
decodable form. Only a re-recording with more frequent keyframes fixes it.

### Camera and lidar don't line up in time
`foxglove_msgs/CompressedVideo.timestamp` is a monotonic clock (~407 s at the
start), not Unix time. Match cameras to lidar by message **`log_time`** instead
(both scripts already do this).

### `no /tf_static path from 'X' to 'CABIN'`
The requested `--target-frame` isn't reachable using only static transforms.
`BASE` is the classic case — it's joined to `CABIN` through the dynamic `/tf`
(cabin slew). Target `CABIN` or a `livox_*` frame, or extend the script to read
`/tf` at each lidar timestamp.

### 3D fusion looks doubled / rotated / offset
Almost always a transform-direction bug. A ROS transform `(parent, child, R, t)`
maps **child→parent**: `p_parent = R·p_child + t`. To move points from a lidar
frame into the target frame you compose the child→parent transforms along the
path (see `resolve_tf` / `build_tf_graph`). Quick check: render a bird's-eye
scatter of each lidar in a different colour — walls/ground must coincide.

### PCD parses with wrong coordinates in PCL/Open3D
The writer emits `DATA binary` with contiguous `float32 x y z intensity`
(16 B/point), `WIDTH = POINTS`, `HEIGHT 1`. If a reader wants ASCII, convert with
Open3D (`o3d.io.read_point_cloud` → `write_point_cloud(..., write_ascii=True)`).

### 3D task imports but contextual images don't appear
CVAT's documented example uses `related_images/*_pcd/*.png`; this toolkit writes
`.jpg` (accepted by the Datumaro point-cloud reader and by CVAT's "default
option 2" layout). If a given CVAT build ignores them, re-run without
`--no-context` but converting frames to PNG (edit `mcap_to_cvat_3d.py`:
`im.save(b, "PNG")` and name the file `.png`). Expect the zip to grow ~3×.
Also check the folder name is exactly `<pcd-stem>_pcd` (e.g. `000000_pcd`).

### Upload rejected for size (video or zip)
* Video: raise `--crf` (e.g. 26), or `--preset veryfast`, or use `--per-camera`.
* 3D zip: `--img-maxdim 960 --jpeg-quality 80`, or split into frame ranges, or
  use CVAT cloud storage.

### `context topic 'C:/Program Files/Git/...': no messages`
Git Bash (MSYS) rewrote a `/hal/...` topic argument into a Windows path. Prefix
the command with `MSYS2_ARG_CONV_EXCL='*'` (and/or `MSYS_NO_PATHCONV=1`), or run
the script from PowerShell / cmd where no rewriting happens.

### `mcap_ros2` can't decode a message
The bag embeds message schemas, so standard types decode without ROS. Truly
custom types without an embedded schema will fail — inspect with
`inspect_mcap.py --schema <name>` and add the definition, or read the raw CDR.

### PyAV / FFmpeg
`pip install av` ships its own FFmpeg with `libx264` and HEVC decode — a system
`ffmpeg` is not used and not required. If `av.CodecContext.create("hevc","r")`
fails, reinstall `av` from a wheel (`pip install --force-reinstall av`).

### Reproducing the outputs
```powershell
py mcap_to_cvat_video.py "BAG.mcap"
py mcap_to_cvat_3d.py "BAG.mcap" --context-topic /hal/perception/Main/compressed_video
```
The 3D script is deterministic; point counts per frame are listed in
`reference/bag_contents.md` for a sanity check (frame 0 = 8269, 300 = 27155,
617 = 27513).
