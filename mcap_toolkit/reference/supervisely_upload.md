# Uploading to Supervisely (supervisely.com)

`mcap_to_supervisely_pce.py` writes a **Point Cloud Episodes** project as a zip.

## Import

1. In Supervisely: **Import Data** → **Quick Auto Import** (or the
   *Import pointcloud episode* app).
2. Drag the zip (or its unpacked folder) onto the drop zone → **Select Files**.
3. Auto-detect should report **Point clouds format: `.pcd`** and, if present,
   pick up the photo-context images. Confirm / start.
4. Open the project → the episode → the 3D labeling tool. Cuboid classes from
   `--classes` are already in the project; the camera images show in the
   photo-context / sensor-fusion panel (one per camera).

## Zip layout produced

```
<project>/
├── meta.json                       classes (cuboid_3d) + tags
├── key_id_map.json                 empty maps
└── <episode>/
    ├── annotation.json             empty episode (framesCount set, no figures)
    ├── frame_pointcloud_map.json   {"0":"000000.pcd", ...}
    ├── pointcloud/
    │   ├── 000000.pcd              binary PCD v0.7, float32 x y z intensity
    │   └── ...
    └── related_images/             (only with --context-topics)
        └── 000000_pcd/             folder = "<pcd stem>_pcd"
            ├── 0_Main.jpg
            ├── 0_Main.jpg.json     {"name": "...", "meta": {"deviceId": "Main"}}
            ├── 1_MastLeftSide.jpg
            ├── 1_MastLeftSide.jpg.json
            └── ...                 (one image + sidecar per camera)
```

## Camera video (2D) under a 25 MB cap

The full 5-camera MP4 is ~460 MB, so for a 25 MB ceiling use
`compress_video_for_size.py` (frame-decimate + resize + best-fit CRF):

```powershell
py compress_video_for_size.py "<bag>_cvat.mp4" --out "<bag>_supervisely.mp4" --max-mb 24 --stride 10 --size 1024x1024
```

* Output: every 10th frame (2.5 fps), 1024×1024, H.264 — **22.5 MB / 21.4 MiB**, so it
  fits whether the cap is read as 25 MB or 25 MiB. All 5 cameras stay in, 170
  frames each; the `_segments.csv` gives the frame range per camera.
* Note the "25 MB" figure in the Quick Auto Import dialog is labelled *per cloud*
  (point clouds). Whether Supervisely applies a cap to video files is not shown
  there; the compressed video is built to fit either way.
* Want more frames? `--stride 5 --size 768x768` keeps 5 fps at lower resolution.

## Several bags

One zip = one project = one episode. Give each bag its own episode name
(`--episode <bag stem>`, `--project <bag stem>_pce`) so the datasets stay distinct
if you later import more than one bag into the same Supervisely project.
Built 2026-09-21 (first 40 frames, 5 cameras, 640 px), in `supervisely_output\`:
`rosbag2_2026_09_15-06_59_27_0_pce.zip` (33.4 MB) and
`rosbag2_2026_09_15-23_06_38_0_pce.zip` (29.4 MB). Largest single file in each: 0.43 MB.

## Notes / limits

* **Cloud size limit** in the import dialog is **per `.pcd` file**, not per
  project: Free 25 MB, Pro 100 MB, Pro+Max 500 MB. Our clouds are ~0.4–0.8 MB
  each, so any plan is fine.
* Points are in the **CABIN** frame (`+x` fwd, `+y` left, `+z` up), ground
  ≈ `z = −2 m`. Supervisely cuboid geometry uses the same x-fwd / y-left / z-up
  convention.
* The photo-context sidecars carry full calibration by default —
  `meta.sensorsData.intrinsicMatrix` (row-major 3×3, from the bag's
  `*/camera_info`, scaled to match the saved JPEG's actual size) and
  `extrinsicMatrix` (row-major 3×4 world→camera, inverted from the
  camera→CABIN `/tf_static` transform) — so cuboids drawn in 3D project onto
  every camera panel, like Supervisely's own nuScenes demo project. Verified by
  reprojecting a built point cloud onto its own camera image: points trace the
  ground ridges and stop exactly at object silhouettes, no drift. Pass
  `--no-calib` to fall back to reference-only images (just `deviceId`, no
  projection) — e.g. if a camera has no `camera_info` topic or no `/tf_static`
  path to the target frame (the tool logs which cameras that affects).
* PCD encoding: Supervisely accepts ascii / binary / binary_compressed; this
  writes plain binary.
* A slice for testing: `--start` / `--count` (e.g. `--count 40` = first ~4 s).
  Whole bag: omit both.
* Supervisely can also ingest a **bare folder of `.pcd` files** with no JSON at
  all (imports as a Point Clouds project, no episode ordering, no photo context).
  Use `--no-context` and just upload the `pointcloud/` folder if you want the
  absolute minimum test.
