# Uploading to CVAT (app.cvat.ai)

Two separate tasks. CVAT can't mix 2D (images/video) and 3D (point clouds) in one
task — the camera video and the point-cloud zip are independent uploads.

Reference: <https://docs.cvat.ai/docs/workspace/tasks-page/#create-annotation-task>

---

## 2D task — the camera video

1. **Tasks → + Create new task**.
2. **Name** — e.g. `rosbag 2026-07-14 cameras`.
   (Leave **Project** blank unless you already have one; a project overrides task
   labels with its own.)
3. **Labels → Constructor → Add label** — one per class, e.g. `person`,
   `excavator`, `wheel_loader`, `truck`, `machine`, `cone`. Set **Label Shape**
   to `Rectangle` for detection (or `Any`). Add attributes if needed
   (immutable = constant per object, mutable = can change per frame).
4. **Select files → My computer** → `…_cvat.mp4`.
5. **Advanced configuration**:
   | Option | Set to | Why |
   |---|---|---|
   | Prefer zip chunks | **off** | docs recommend off for video — less traffic |
   | Use cache | on (default) | on-the-fly chunking |
   | Image quality | **90–100** | gravel-pit people/objects are small at distance; don't over-compress |
   | Chunk size | leave blank (auto) or `8–16` | frame is ~3.7 MP (between 1080p→36 and 2K→8) |
   | Start/Stop frame, Frame step | leave default | the MP4 is already exactly the frames you want |
   | Segment size / Overlap | leave blank unless splitting work | see note below |
6. **Submit & Open**. CVAT decodes H.264 server-side. Frames are 0-based,
   constant 25 fps.
7. Camera → frame range (`frame_segments.csv`):
   Main `0–1482` · MastLeftRear `1483–2965` · MastLeftSide `2966–4478` ·
   MastRightRear `4479–5991` · MastRightSide `5992–7504`.

**Segment size note:** CVAT segments are arbitrary frame chunks for parallel
annotators — they do **not** follow the 5 camera boundaries. If you set a segment
size, a chunk may straddle two cameras; that's cosmetic (just a jump in the
scene). Overlap only helps bounding-box track continuity and only if boundaries
align. Simplest: leave both blank and annotate as one segment.

Five independent tasks instead: re-run with `--per-camera` and upload each MP4.
In the **Name** field you can use `{{file_name}}` / `{{index}}` for multi-task
creation (video only).

---

## 3D task — the fused point cloud

1. **Tasks → + Create new task**.
2. **Name** — e.g. `rosbag 2026-07-14 lidar 3D`.
3. **Labels** — 3D annotates **cuboids**; add `person`, `excavator`, `truck`, …
   (Label Shape `Cuboid` or `Any`).
4. **Select files → My computer** → the whole `…_cvat_3d.zip`.
   It is already in CVAT's **"3D Pointcloud Format"**:
   ```
   pointcloud/000000.pcd, 000001.pcd, …
   related_images/000000_pcd/0_Main.jpg
   related_images/000000_pcd/1_MastLeftSide.jpg
   related_images/000000_pcd/2_MastRightSide.jpg
   related_images/000000_pcd/3_MastLeftRear.jpg
   related_images/000000_pcd/4_MastRightRear.jpg
   frame_index.csv            (ignored by CVAT; reference only)
   ```
   Each `_pcd/` folder holds the 5 cameras' time-matched frames as **separate
   images**, so CVAT shows one **contextual-image panel per camera** in the 3D
   workspace (open the context panel from the right-hand sidebar; drag to
   rearrange, click to enlarge). `frame_index.csv` `cam_frames` column lists the
   exact source frame each panel came from.
   CVAT's other accepted 3D layouts (for reference): Velodyne
   (`velodyne_points/data/*.bin` + `data/*.png`), and two "default" single-frame
   layouts under `data/`.
5. **Advanced configuration** — defaults are fine. Image quality affects the
   contextual JPEGs only.
6. **Submit & Open**. The task has 618 frames. The camera JPEG appears in the
   contextual-image panel next to the 3D view.
   Controls: `Shift`+drag rotate · scroll zoom · draw a cuboid then refine it in
   the top/side/front projection panes.
7. `frame_index.csv` inside the zip: per frame → reference timestamp, lidars
   merged (`FL+RL+RR` / `FL+RL` / `FL`), point count, matched Main-camera frame.

### Notes / limits

* Points are in the **CABIN** frame: `+x` ≈ forward, `+z` up, origin in the cab,
  ground ≈ `z = −2 m`. The world rotates around the machine when the cabin slews —
  normal for ego-centric 3D annotation.
* `.pcd` = binary `x y z intensity` float32, PCD v0.7 (Open3D / PCL / CloudCompare
  / Three.js `PCDLoader` all read it).
* CVAT's format example writes `related_images/*_pcd/*.png`; `.jpg` also works
  (the Datumaro point-cloud reader accepts any image extension, and CVAT's own
  "default option 2" layout lists `{png|jpg}`). If contextual images don't show
  after import, re-run with PNGs — but that inflates the zip ~3×.
* Contextual images are **reference only** — not projected into 3D, no camera
  calibration embedded. Confirmed by reading CVAT's actual importer source (the
  `sly_pointcloud` and `kitti_raw`/Velodyne formats it uses, plus the
  `cvat-canvas3d` frontend): neither reads any calibration for related images,
  they're globbed as plain files. No setting changes that — unlike Supervisely
  (`supervisely_upload.md`), CVAT has no cuboid-onto-image projection at all.
* Upload is ~500 MB; the docs list no hard size limit. If app.cvat.ai rejects it:
  re-run with `--img-maxdim 960 --jpeg-quality 80`, or split into frame ranges
  (keep `pointcloud/` and `related_images/` in sync), or attach from
  **Cloud Storage** (note: multi-task creation is not available from cloud
  storage).
