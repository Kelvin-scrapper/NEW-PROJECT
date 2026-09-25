"""
mcap_to_supervisely_pce.py - fuse the LiDAR streams in a rosbag2 .mcap into a
**Supervisely Point Cloud Episodes** project (optionally a short slice for a
quick test upload).

Output layout (zipped), per the Supervisely PCE spec:

    <project>/
      meta.json
      key_id_map.json
      <episode>/
        annotation.json
        frame_pointcloud_map.json
        pointcloud/000000.pcd ...
        related_images/000000_pcd/            (with --context-topics)
          0_Main.jpg
          0_Main.jpg.json                     photo-context sidecar (+ calibration, see below)
          1_MastLeftSide.jpg
          1_MastLeftSide.jpg.json
          ...

Point-cloud fusion is identical to mcap_to_cvat_3d.py (shared code): all
PointCloud2 topics merged per reference frame into --target-frame via /tf_static,
float32 x y z intensity binary PCD.

Camera calibration (on by default, for cuboid -> image projection in Supervisely)
-----------------------------------------------------------------------------
For each --context-topics camera, its `*/camera_info` sibling topic (same topic
with the last path segment replaced by "camera_info") gives the intrinsic K
(row-major 3x3, used as-is). Its `header.frame_id` is resolved through
/tf_static to --target-frame for the camera->world rigid transform, which is
inverted to the world->camera transform Supervisely wants
(meta.sensorsData.extrinsicMatrix, row-major 3x4 [R|t]). A camera missing its
camera_info topic or TF path gets a sidecar with no sensorsData (image still
shows, just not projectable) - use --no-calib to skip this for every camera.

Quick test slice:  --start 0 --count 40   -> first 40 frames only.

Examples
--------
    py mcap_to_supervisely_pce.py BAG.mcap --count 40 --img-maxdim 640 \
        --context-topics /hal/perception/Main/compressed_video \
                         /hal/perception/MastLeftSide/compressed_video \
                         /hal/perception/MastRightSide/compressed_video \
                         /hal/perception/MastLeftRear/compressed_video \
                         /hal/perception/MastRightRear/compressed_video

    py mcap_to_supervisely_pce.py BAG.mcap --no-context          # whole bag, clouds only
"""
import argparse
import json
import shutil
import time
import uuid
import zipfile
from pathlib import Path

import io

import numpy as np
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory
from PIL import Image

import mcap_to_cvat_3d as c3   # build_tf_graph, resolve_tf, cloud_dtype, parse_cloud,
                               # write_pcd, prep_camera, as_bytes, short_name

CLASS_COLORS = ["#FF3838", "#FF9D00", "#3888FF", "#00C21F", "#B65FFF", "#00B8D9",
                "#FFB6C1", "#8B4513", "#FFD700", "#7CFC00"]


def log(m):
    print(f"[{time.strftime('%H:%M:%S')}] {m}", flush=True)


def camera_info_topic(video_topic):
    parts = video_topic.rsplit("/", 1)
    return parts[0] + "/camera_info" if len(parts) == 2 else None


def read_camera_infos(reader, video_topics):
    """{video_topic: (frame_id, K[9], width, height)} - native camera resolution;
    for whichever cameras have a camera_info sibling."""
    wanted = {t: camera_info_topic(t) for t in video_topics}
    by_ci_topic = {v: k for k, v in wanted.items() if v}
    out = {}
    for _s, ch, _m, ros in reader.iter_decoded_messages(topics=list(by_ci_topic)):
        vt = by_ci_topic.get(ch.topic)
        if vt and vt not in out:
            out[vt] = (ros.header.frame_id, [float(x) for x in ros.k], ros.width, ros.height)
        if len(out) == len(by_ci_topic):
            break
    return out


def scale_intrinsics(K, orig_w, orig_h, actual_w, actual_h):
    """K' = diag(sx,sy,1) @ K, for the photo-context JPEG's actual (downscaled) size."""
    sx, sy = actual_w / orig_w, actual_h / orig_h
    fx, skew, cx, _, fy, cy, *_ = K
    return [fx * sx, skew * sx, cx * sx, 0.0, fy * sy, cy * sy, 0.0, 0.0, 1.0]


def camera_extrinsics(tf_graph, cam_frame_ids, target_frame):
    """{video_topic: extrinsicMatrix} - row-major 3x4 [R|t] mapping a point in
    `target_frame` into that camera's optical frame (Supervisely's world->camera)."""
    out = {}
    for topic, frame_id in cam_frame_ids.items():
        try:
            R, t = c3.resolve_tf(tf_graph, frame_id, target_frame)   # camera -> target
        except SystemExit:
            continue
        Rinv = R.T
        tinv = -R.T @ t                                             # target -> camera
        out[topic] = [
            Rinv[0, 0], Rinv[0, 1], Rinv[0, 2], tinv[0],
            Rinv[1, 0], Rinv[1, 1], Rinv[1, 2], tinv[1],
            Rinv[2, 0], Rinv[2, 1], Rinv[2, 2], tinv[2],
        ]
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag", type=Path)
    ap.add_argument("--out", type=Path, help="output .zip (default: <bag dir>/supervisely_output/<project>.zip)")
    ap.add_argument("--project", help="project folder name (default: <bag stem>_pce)")
    ap.add_argument("--episode", default="episode_1", help="episode/dataset folder name")
    ap.add_argument("--lidar-topics", nargs="+")
    ap.add_argument("--ref", help="reference lidar topic (default: most messages)")
    ap.add_argument("--target-frame", default="CABIN")
    ap.add_argument("--match-tol", type=float, default=0.06)
    ap.add_argument("--start", type=int, default=0, help="first reference-frame index to export")
    ap.add_argument("--count", type=int, default=0, help="number of frames (0 = to the end)")
    ap.add_argument("--context-topics", nargs="+", default=[], help="camera topics -> photo context per frame")
    ap.add_argument("--no-context", action="store_true")
    ap.add_argument("--no-calib", action="store_true",
                    help="skip camera_info/tf_static calibration in the sidecars (no cuboid->image projection)")
    ap.add_argument("--img-maxdim", type=int, default=640, help="photo-context image longest side")
    ap.add_argument("--jpeg-quality", type=int, default=80)
    ap.add_argument("--classes", default="person,excavator,wheel_loader,truck,car,machine",
                    help="comma-separated cuboid_3d class titles for meta.json")
    ap.add_argument("--keep-stage", action="store_true")
    args = ap.parse_args()

    ctx = [] if args.no_context else list(dict.fromkeys(args.context_topics))
    project = args.project or (args.bag.stem + "_pce")
    out_zip = args.out or args.bag.parent / "supervisely_output" / (project + ".zip")
    out_zip.parent.mkdir(parents=True, exist_ok=True)
    stage = out_zip.parent / ("_stage_" + out_zip.stem)
    if stage.exists():
        shutil.rmtree(stage)
    P = stage / project
    DS = P / args.episode
    (DS / "pointcloud").mkdir(parents=True)
    if ctx:
        (DS / "related_images").mkdir()

    with open(args.bag, "rb") as f:
        reader = make_reader(f, decoder_factories=[DecoderFactory()])
        summary = reader.get_summary()
        schemas = {s.id: s for s in summary.schemas.values()}
        counts = summary.statistics.channel_message_counts

        pc_topics = args.lidar_topics or sorted(
            ch.topic for ch in summary.channels.values()
            if schemas.get(ch.schema_id) and schemas[ch.schema_id].name == "sensor_msgs/msg/PointCloud2")
        if not pc_topics:
            raise SystemExit("no sensor_msgs/msg/PointCloud2 topics found")
        ch_by = {ch.topic: ch for ch in summary.channels.values()}
        ref = args.ref or max(pc_topics, key=lambda t: counts.get(ch_by[t].id, 0))
        others = [t for t in pc_topics if t != ref]
        log(f"lidars: {pc_topics}")
        log(f"reference: {ref}   target frame: {args.target_frame}")

        tf_graph = c3.build_tf_graph(reader)

        calib = {}
        if ctx and not args.no_calib:
            cam_infos = read_camera_infos(reader, ctx)
            cam_frame_ids = {t: fid for t, (fid, _k, _w, _h) in cam_infos.items()}
            extr = camera_extrinsics(tf_graph, cam_frame_ids, args.target_frame)
            for t in ctx:
                if t in cam_infos and t in extr:
                    _fid, K, ow, oh = cam_infos[t]
                    calib[t] = {"K": K, "orig_w": ow, "orig_h": oh, "extrinsic": extr[t]}
            missing = [c3.short_name(t) for t in ctx if t not in calib]
            log(f"calibration: {len(calib)}/{len(ctx)} cameras "
                f"(missing camera_info and/or tf_static path: {missing or 'none'})")

        raw = {t: [] for t in pc_topics}
        for _s, ch, _m, ros in reader.iter_decoded_messages(topics=pc_topics):
            dt, has_i = c3.cloud_dtype(ros.fields, ros.point_step)
            raw[ch.topic].append((ros.header.stamp.sec + ros.header.stamp.nanosec / 1e9,
                                  c3.as_bytes(ros.data), ros.width * ros.height, dt, has_i))
        for t in raw:
            raw[t].sort(key=lambda r: r[0])

        frame_ids = {}
        for _s, ch, _m, ros in reader.iter_decoded_messages(topics=pc_topics):
            frame_ids.setdefault(ch.topic, ros.header.frame_id)
            if len(frame_ids) == len(pc_topics):
                break
        TF = {t: c3.resolve_tf(tf_graph, frame_ids[t], args.target_frame) for t in pc_topics}
        other_ts = {t: np.array([r[0] for r in raw[t]]) for t in others}

        ref_all = raw[ref]
        s = max(0, args.start)
        e = len(ref_all) if args.count <= 0 else min(len(ref_all), s + args.count)
        sel = ref_all[s:e]
        log(f"exporting reference frames {s}..{e - 1}  ({len(sel)} of {len(ref_all)})")

        cam_ts, cam_jpg = {}, {}
        for c in ctx:
            cam_ts[c], cam_jpg[c] = c3.prep_camera(reader, c, False, (0, 0),
                                                   args.img_maxdim, args.jpeg_quality)

        def nearest(ts, x):
            if len(ts) == 0:
                return None
            i = int(np.clip(np.searchsorted(ts, x), 1, len(ts) - 1))
            j = i - 1 if abs(ts[i - 1] - x) <= abs(ts[i] - x) else i
            return j if abs(ts[j] - x) <= args.match_tol else None

        fmap = {}
        t0 = time.time()
        for oi, (stamp, rb, n, dt, hi) in enumerate(sel):
            xs, is_ = [], []
            R, t = TF[ref]
            xyz, inten = c3.parse_cloud(rb, n, dt, hi)
            xs.append(xyz @ R.T + t); is_.append(inten)
            for ot in others:
                j = nearest(other_ts[ot], stamp)
                if j is None:
                    continue
                _s2, rb2, n2, dt2, hi2 = raw[ot][j]
                xyz, inten = c3.parse_cloud(rb2, n2, dt2, hi2)
                R, t = TF[ot]
                xs.append(xyz @ R.T + t); is_.append(inten)
            mx = np.concatenate(xs); mi = np.concatenate(is_)
            pcd_name = f"{oi:06d}.pcd"
            c3.write_pcd(DS / "pointcloud" / pcd_name, mx, mi)
            fmap[str(oi)] = pcd_name

            if ctx:
                d = DS / "related_images" / f"{oi:06d}_pcd"
                d.mkdir(exist_ok=True)
                for k, c in enumerate(ctx):
                    jj = int(np.argmin(np.abs(cam_ts[c] - stamp)))
                    img_name = f"{k}_{c3.short_name(c)}.jpg"
                    jpg_bytes = cam_jpg[c][jj]
                    (d / img_name).write_bytes(jpg_bytes)
                    meta = {"deviceId": c3.short_name(c)}
                    if c in calib:
                        cal = calib[c]
                        aw, ah = Image.open(io.BytesIO(jpg_bytes)).size
                        K_scaled = scale_intrinsics(cal["K"], cal["orig_w"], cal["orig_h"], aw, ah)
                        meta["sensorsData"] = {"extrinsicMatrix": cal["extrinsic"],
                                               "intrinsicMatrix": K_scaled}
                    (d / f"{img_name}.json").write_text(json.dumps(
                        {"name": img_name, "meta": meta}, indent=2))
            if (oi + 1) % 50 == 0:
                log(f"  {oi + 1}/{len(sel)} frames ({(oi + 1) / (time.time() - t0):.0f}/s)")

    classes = [{"title": c.strip(), "shape": "cuboid_3d",
                "color": CLASS_COLORS[i % len(CLASS_COLORS)], "geometry_config": {}}
               for i, c in enumerate(args.classes.split(",")) if c.strip()]
    (P / "meta.json").write_text(json.dumps(
        {"classes": classes, "tags": [], "projectType": "point_cloud_episodes"}, indent=2))
    (P / "key_id_map.json").write_text(json.dumps(
        {"tags": {}, "objects": {}, "figures": {}, "videos": {}}, indent=2))
    (DS / "frame_pointcloud_map.json").write_text(json.dumps(fmap, indent=2))
    (DS / "annotation.json").write_text(json.dumps(
        {"description": "", "key": uuid.uuid4().hex, "tags": [],
         "objects": [], "framesCount": len(fmap), "frames": []}, indent=2))

    log("zipping ...")
    nfiles = 0
    with zipfile.ZipFile(out_zip, "w", zipfile.ZIP_STORED) as z:
        for p in sorted(stage.rglob("*")):
            if p.is_file():
                z.write(p, p.relative_to(stage).as_posix())
                nfiles += 1
    if not args.keep_stage:
        shutil.rmtree(stage)
    log(f"WROTE {out_zip}  ({out_zip.stat().st_size / 1e6:.1f} MB, {nfiles} files, {len(fmap)} frames)")
    log("ALL DONE")


if __name__ == "__main__":
    main()
