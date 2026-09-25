"""
mcap_to_cvat_3d.py - fuse the LiDAR PointCloud2 streams in a rosbag2 .mcap into a
CVAT 3D point-cloud task (one merged .pcd per frame + optional contextual camera
image), zipped in the CVAT "3D Pointcloud" layout:

    pointcloud/000000.pcd ...
    related_images/000000_pcd/000000.jpg ...   (with --context-topic[s])
    frame_index.csv

How the fusion works
--------------------
* All PointCloud2 topics are auto-detected (override with --lidar-topics).
* One topic is the reference clock (--ref, default = the one with most messages).
  Its timestamps define the output frames.
* Every other lidar is nearest-timestamp matched to each reference frame within
  --match-tol seconds, transformed into --target-frame using ONLY /tf_static
  (rigid mounts), and concatenated.
* Point fields x/y/z plus the first intensity-like field (intensity | reflectivity
  | i) are kept and written as float32 x y z intensity.

Camera context
--------------
CVAT shows every image found in `related_images/<frame>_pcd/` as its own panel.
  --context-topic  T                one camera  -> one image per lidar frame
  --context-topics T1 T2 ...        several cameras -> one image EACH per lidar
                                    frame (separate CVAT panels), named
                                    <n>_<camera>.jpg in CLI order
  --composite                       instead tile them into ONE grid image
                                    (--context-cols wide, --cell WxH)
CompressedVideo (h264/h265) and CompressedImage (jpeg/png) are supported. Camera
frames are matched to each lidar frame by bag log_time (robust to cameras that
publish a header stamp on a different clock).

Examples
--------
    py mcap_to_cvat_3d.py BAG.mcap
    py mcap_to_cvat_3d.py BAG.mcap --context-topic /hal/perception/Main/compressed_video
    py mcap_to_cvat_3d.py BAG.mcap --img-maxdim 960 --context-topics \
        /hal/perception/Main/compressed_video \
        /hal/perception/MastLeftSide/compressed_video \
        /hal/perception/MastRightSide/compressed_video \
        /hal/perception/MastLeftRear/compressed_video \
        /hal/perception/MastRightRear/compressed_video
    py mcap_to_cvat_3d.py BAG.mcap --lidar-topics /livox/a /livox/b --no-context
"""
import argparse
import csv
import io
import shutil
import time
import zipfile
from collections import deque
from pathlib import Path

import numpy as np
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory
from PIL import Image, ImageDraw, ImageFont
from scipy.spatial.transform import Rotation

PF = {1: "<i1", 2: "<u1", 3: "<i2", 4: "<u2", 5: "<i4", 6: "<u4", 7: "<f4", 8: "<f8"}
INTENSITY_NAMES = ("intensity", "reflectivity", "i", "reflect")


def log(m):
    print(f"[{time.strftime('%H:%M:%S')}] {m}", flush=True)


def as_bytes(x):
    return bytes(x) if isinstance(x, (bytes, bytearray, memoryview)) else bytes(bytearray(x))


def get_font(sz=22):
    for name in ("arialbd.ttf", "arial.ttf", "DejaVuSans-Bold.ttf", "DejaVuSans.ttf"):
        try:
            return ImageFont.truetype(name, sz)
        except Exception:
            continue
    return ImageFont.load_default()


FONT = get_font()


def short_name(topic):
    parts = [p for p in topic.split("/") if p]
    return parts[-2] if len(parts) >= 2 else (parts[-1] if parts else topic)


# ----------------------------------------------------------------------------- TF
def build_tf_graph(reader):
    """Adjacency map from /tf_static. The edge stored under key `f` toward
    neighbour `nb` carries (R, t) that maps a point in frame `f` INTO frame `nb`:
    p_nb = R @ p_f + t.

    A ROS transform (parent=p, child=ch, rot=q, trans=tv) means p_p = Rq @ p_ch + tv
    i.e. it maps ch-coords -> p-coords. So the ch->p edge is (Rq, tv) and the
    p->ch edge is its inverse (Rq.T, -Rq.T @ tv).
    """
    g = {}
    for _s, _c, _m, ros in reader.iter_decoded_messages(topics=["/tf_static"]):
        for tr in ros.transforms:
            p, ch = tr.header.frame_id, tr.child_frame_id
            q, t = tr.transform.rotation, tr.transform.translation
            R = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
            tv = np.array([t.x, t.y, t.z], float)
            g.setdefault(ch, []).append((p, R, tv))                 # ch -> p
            g.setdefault(p, []).append((ch, R.T, -R.T @ tv))        # p  -> ch
        break
    return g


def resolve_tf(graph, src, dst):
    """Rigid transform (R, t) mapping a point in frame `src` into frame `dst`:
    p_dst = R @ p_src + t."""
    if src == dst:
        return np.eye(3), np.zeros(3)
    seen = {src}
    q = deque([(src, np.eye(3), np.zeros(3))])
    while q:
        f, R, t = q.popleft()
        for nb, Rn, tn in graph.get(f, ()):
            if nb in seen:
                continue
            R2, t2 = Rn @ R, Rn @ t + tn          # p_nb = Rn @ (R @ p_src + t) + tn
            if nb == dst:
                return R2, t2
            seen.add(nb)
            q.append((nb, R2, t2))
    raise SystemExit(f"no /tf_static path from {src!r} to {dst!r}")


# -------------------------------------------------------------------- pointcloud
def cloud_dtype(fields, point_step):
    """Structured numpy dtype covering x/y/z (+ intensity-like) at their byte
    offsets inside one point of `point_step` bytes."""
    have = {f.name: f for f in fields}
    names, formats, offsets = [], [], []
    for ax in ("x", "y", "z"):
        f = have[ax]
        names.append(ax); formats.append(PF[f.datatype]); offsets.append(f.offset)
    inten = next((have[n] for n in INTENSITY_NAMES if n in have), None)
    if inten is not None:
        names.append("intensity"); formats.append(PF[inten.datatype]); offsets.append(inten.offset)
    dt = np.dtype({"names": names, "formats": formats, "offsets": offsets,
                   "itemsize": point_step})
    return dt, inten is not None


def parse_cloud(raw, n, dt, has_i):
    rec = np.frombuffer(raw, dtype=dt, count=n)
    xyz = np.stack([rec["x"], rec["y"], rec["z"]], 1).astype(np.float64)
    inten = rec["intensity"].astype(np.float32) if has_i else np.zeros(n, np.float32)
    ok = np.isfinite(xyz).all(1)
    return xyz[ok], inten[ok]


def write_pcd(path, xyz, inten):
    n = len(xyz)
    hdr = ("# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\n"
           "FIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n"
           f"WIDTH {n}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {n}\nDATA binary\n")
    body = np.empty((n, 4), np.float32)
    body[:, :3] = xyz
    body[:, 3] = inten
    with open(path, "wb") as f:
        f.write(hdr.encode("ascii"))
        f.write(body.tobytes())


# ------------------------------------------------------------------------ camera
def decode_camera(reader, topic):
    """-> (log_times_ns list, generator_factory) ; frames are RGB uint8 ndarrays."""
    import av
    msgs = []
    schema_name = None
    for s, _c, m, ros in reader.iter_decoded_messages(topics=[topic]):
        schema_name = s.name
        msgs.append((m.log_time, ros))
    if not msgs:
        raise SystemExit(f"context topic {topic!r}: no messages")
    msgs.sort(key=lambda x: x[0])
    log_times = [t for t, _ in msgs]

    if schema_name == "foxglove_msgs/msg/CompressedVideo":
        fmt = msgs[0][1].format.lower()
        codec = {"h264": "h264", "h265": "hevc", "hevc": "hevc"}[fmt]
        blob = b"".join(as_bytes(r.data) for _t, r in msgs)

        def gen():
            dec = av.CodecContext.create(codec, "r")
            for pkt in dec.parse(blob):
                for fr in dec.decode(pkt):
                    yield fr.to_ndarray(format="rgb24")
            for fr in dec.decode(None):
                yield fr.to_ndarray(format="rgb24")
        return log_times, gen
    if schema_name == "sensor_msgs/msg/CompressedImage":
        import cv2

        def gen():
            for _t, r in msgs:
                a = cv2.imdecode(np.frombuffer(as_bytes(r.data), np.uint8), cv2.IMREAD_COLOR)
                yield a[:, :, ::-1].copy()
        return log_times, gen
    raise SystemExit(f"context topic {topic}: unsupported schema {schema_name}")


def letterbox_jpeg(rgb, box, quality):
    """Fit an RGB ndarray inside box=(w,h) keeping aspect, pad black -> JPEG bytes."""
    bw, bh = box
    im = Image.fromarray(rgb)
    im.thumbnail((bw, bh))
    canvas = Image.new("RGB", (bw, bh), (0, 0, 0))
    canvas.paste(im, ((bw - im.width) // 2, (bh - im.height) // 2))
    b = io.BytesIO()
    canvas.save(b, "JPEG", quality=quality)
    return b.getvalue()


def prep_camera(reader, topic, composite, cell, img_maxdim, quality):
    """Decode a camera once; return (ts_seconds_per_frame, list_of_jpeg_bytes).
    In composite mode each jpeg is letterboxed to `cell`; otherwise thumbnailed
    to img_maxdim."""
    log_times, genfac = decode_camera(reader, topic)
    jpegs = []
    for rgb in genfac():
        if composite:
            jpegs.append(letterbox_jpeg(rgb, cell, quality))
        else:
            im = Image.fromarray(rgb)
            im.thumbnail((img_maxdim, img_maxdim))
            b = io.BytesIO()
            im.save(b, "JPEG", quality=quality)
            jpegs.append(b.getvalue())
    offset = len(log_times) - len(jpegs)
    ts = np.array(log_times[offset:], float) / 1e9
    log(f"context {short_name(topic)}: {len(jpegs)} frames (offset {offset})")
    return ts, jpegs


def composite_image(cells, topics, cols, cell):
    """cells: {topic: jpeg bytes}. -> composite JPEG-ready PIL image."""
    cw, ch = cell
    rows = (len(topics) + cols - 1) // cols
    canvas = Image.new("RGB", (cols * cw, rows * ch), (12, 12, 12))
    d = ImageDraw.Draw(canvas)
    for idx, tp in enumerate(topics):
        r, c = divmod(idx, cols)
        x0, y0 = c * cw, r * ch
        if tp in cells:
            canvas.paste(Image.open(io.BytesIO(cells[tp])), (x0, y0))
        name = short_name(tp)
        tb = d.textbbox((x0 + 6, y0 + 4), name, font=FONT)
        d.rectangle([tb[0] - 4, tb[1] - 3, tb[2] + 4, tb[3] + 3], fill=(0, 0, 0))
        d.text((x0 + 6, y0 + 4), name, font=FONT, fill=(255, 255, 0))
    return canvas


# -------------------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag", type=Path)
    ap.add_argument("--out", type=Path, help="output .zip (default: <bag dir>/cvat_output/<stem>_cvat_3d.zip)")
    ap.add_argument("--lidar-topics", nargs="+")
    ap.add_argument("--ref", help="reference lidar topic (default: most messages)")
    ap.add_argument("--target-frame", default="CABIN")
    ap.add_argument("--match-tol", type=float, default=0.06)
    ap.add_argument("--context-topic", help="one camera topic for the contextual image")
    ap.add_argument("--context-topics", nargs="+",
                    help="several camera topics -> one image each per frame (separate panels)")
    ap.add_argument("--composite", action="store_true",
                    help="tile --context-topics into one grid image instead of separate panels")
    ap.add_argument("--context-cols", type=int, default=3, help="composite grid width")
    ap.add_argument("--cell", default="640x480", help="composite cell size WxH")
    ap.add_argument("--no-context", action="store_true")
    ap.add_argument("--img-maxdim", type=int, default=1280, help="per-camera image longest side")
    ap.add_argument("--jpeg-quality", type=int, default=85)
    ap.add_argument("--keep-stage", action="store_true")
    args = ap.parse_args()

    ctx_topics = []
    if not args.no_context:
        ctx_topics += list(args.context_topics or [])
        if args.context_topic:
            ctx_topics.append(args.context_topic)
        ctx_topics = list(dict.fromkeys(ctx_topics))          # de-dupe, keep order
    composite = args.composite and len(ctx_topics) > 1
    cell = tuple(int(v) for v in args.cell.lower().split("x")) if composite else (0, 0)

    out_zip = args.out or args.bag.parent / "cvat_output" / (args.bag.stem + "_cvat_3d.zip")
    out_zip.parent.mkdir(parents=True, exist_ok=True)
    stage = out_zip.parent / ("_stage_" + out_zip.stem)
    if stage.exists():
        shutil.rmtree(stage)
    (stage / "pointcloud").mkdir(parents=True)
    (stage / "related_images").mkdir()

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
        ch_by_topic = {ch.topic: ch for ch in summary.channels.values()}
        ref = args.ref or max(pc_topics, key=lambda t: counts.get(ch_by_topic[t].id, 0))
        others = [t for t in pc_topics if t != ref]
        log(f"lidars: {pc_topics}")
        log(f"reference: {ref}   target frame: {args.target_frame}")
        if ctx_topics:
            log(f"context: {'composite ' if composite else ''}{[short_name(t) for t in ctx_topics]}")

        tf_graph = build_tf_graph(reader)

        # ---- collect clouds -------------------------------------------------
        raw = {t: [] for t in pc_topics}      # t -> [(stamp_s, bytes, n, dt, has_i)]
        for _s, ch, m, ros in reader.iter_decoded_messages(topics=pc_topics):
            dt, has_i = cloud_dtype(ros.fields, ros.point_step)
            raw[ch.topic].append((ros.header.stamp.sec + ros.header.stamp.nanosec / 1e9,
                                  as_bytes(ros.data), ros.width * ros.height, dt, has_i))
        for t in raw:
            raw[t].sort(key=lambda r: r[0])
        log("counts: " + ", ".join(f"{short_name(t)}={len(v)}" for t, v in raw.items()))

        frame_ids = {}
        for _s, ch, _m, ros in reader.iter_decoded_messages(topics=pc_topics):
            frame_ids.setdefault(ch.topic, ros.header.frame_id)
            if len(frame_ids) == len(pc_topics):
                break
        TF = {t: resolve_tf(tf_graph, frame_ids[t], args.target_frame) for t in pc_topics}
        other_ts = {t: np.array([r[0] for r in raw[t]]) for t in others}

        def nearest(ts, x):
            if len(ts) == 0:
                return None
            i = int(np.clip(np.searchsorted(ts, x), 1, len(ts) - 1))
            j = i - 1 if abs(ts[i - 1] - x) <= abs(ts[i] - x) else i
            return j if abs(ts[j] - x) <= args.match_tol else None

        # ---- camera timelines --------------------------------------------
        cam_ts, cam_jpg = {}, {}
        for c in ctx_topics:
            cam_ts[c], cam_jpg[c] = prep_camera(
                reader, c, composite, cell, args.img_maxdim, args.jpeg_quality)

        # ---- build merged clouds + contextual images --------------------
        rows = []
        t0 = time.time()
        ref_list = raw[ref]
        for i, (stamp, rb, n, dt, hi) in enumerate(ref_list):
            xs, is_, used = [], [], [short_name(ref)]
            R, t = TF[ref]
            xyz, inten = parse_cloud(rb, n, dt, hi)
            xs.append(xyz @ R.T + t); is_.append(inten)
            for ot in others:
                j = nearest(other_ts[ot], stamp)
                if j is None:
                    continue
                _s2, rb2, n2, dt2, hi2 = raw[ot][j]
                xyz, inten = parse_cloud(rb2, n2, dt2, hi2)
                R, t = TF[ot]
                xs.append(xyz @ R.T + t); is_.append(inten)
                used.append(short_name(ot))
            mx = np.concatenate(xs); mi = np.concatenate(is_)
            write_pcd(stage / "pointcloud" / f"{i:06d}.pcd", mx, mi)

            cam_sel = {}
            if ctx_topics:
                d = stage / "related_images" / f"{i:06d}_pcd"
                d.mkdir(exist_ok=True)
                for c in ctx_topics:
                    cam_sel[c] = int(np.argmin(np.abs(cam_ts[c] - stamp)))
                if composite:
                    cells = {c: cam_jpg[c][cam_sel[c]] for c in ctx_topics}
                    composite_image(cells, ctx_topics, args.context_cols, cell).save(
                        d / f"{i:06d}.jpg", "JPEG", quality=args.jpeg_quality)
                elif len(ctx_topics) == 1:
                    c = ctx_topics[0]
                    (d / f"{i:06d}.jpg").write_bytes(cam_jpg[c][cam_sel[c]])
                else:
                    for n, c in enumerate(ctx_topics):
                        (d / f"{n}_{short_name(c)}.jpg").write_bytes(cam_jpg[c][cam_sel[c]])

            rows.append(dict(frame=i, stamp=f"{stamp:.6f}", lidars="+".join(used),
                             points=len(mx),
                             cam_frames="|".join(f"{short_name(c)}:{v}" for c, v in cam_sel.items())))
            if (i + 1) % 100 == 0:
                log(f"  {i+1}/{len(ref_list)} frames ({(i+1)/(time.time()-t0):.0f}/s)")

        with open(stage / "frame_index.csv", "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            w.writeheader(); w.writerows(rows)

    log("zipping ...")
    nfiles = 0
    with zipfile.ZipFile(out_zip, "w", zipfile.ZIP_STORED) as z:
        for p in sorted(stage.rglob("*")):
            if p.is_file():
                z.write(p, p.relative_to(stage).as_posix())
                nfiles += 1
    if not args.keep_stage:
        shutil.rmtree(stage)
    log(f"WROTE {out_zip}  ({out_zip.stat().st_size/1e6:.1f} MB, {nfiles} files)")
    log("ALL DONE")


if __name__ == "__main__":
    main()
