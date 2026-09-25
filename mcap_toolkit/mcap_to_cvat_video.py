"""
mcap_to_cvat_video.py - turn camera streams in a rosbag2 .mcap into H.264 MP4(s)
that upload cleanly to a CVAT 2D task.

Handles these message types (auto-detected):
    foxglove_msgs/msg/CompressedVideo   (h264 / h265 Annex-B)
    sensor_msgs/msg/CompressedImage     (jpeg / png)
    sensor_msgs/msg/Image               (rgb8 / bgr8 / mono8)

Default: every camera is placed (no scaling) on one square canvas and the
segments are concatenated into ONE .mp4 (one CVAT task). Use --per-camera for
one file per topic. A frame_segments.csv mapping is always written.

Examples:
    py mcap_to_cvat_video.py BAG.mcap
    py mcap_to_cvat_video.py BAG.mcap --out out.mp4 --fps 25 --crf 21
    py mcap_to_cvat_video.py BAG.mcap --topics /cam/front /cam/rear --per-camera
"""
import argparse
import csv
import time
from fractions import Fraction
from pathlib import Path

import av
import numpy as np
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory
from PIL import Image, ImageDraw, ImageFont

VIDEO_SCHEMAS = {
    "foxglove_msgs/msg/CompressedVideo",
    "sensor_msgs/msg/CompressedImage",
    "sensor_msgs/msg/Image",
}


def log(msg):
    print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)


def get_font(sz=34):
    for name in ("arialbd.ttf", "arial.ttf", "DejaVuSans-Bold.ttf", "DejaVuSans.ttf"):
        try:
            return ImageFont.truetype(name, sz)
        except Exception:
            continue
    return ImageFont.load_default()


FONT = get_font()


def detect_camera_topics(summary):
    schemas = {s.id: s for s in summary.schemas.values()}
    out = []
    for ch in summary.channels.values():
        sch = schemas.get(ch.schema_id)
        if sch and sch.name in VIDEO_SCHEMAS:
            out.append((ch.topic, sch.name))
    out.sort()
    return out


def frames_from_topic(reader, topic, schema_name):
    """Yield HxWx3 uint8 RGB frames from a camera topic, in log_time order."""
    msgs = []
    for _s, _c, message, ros in reader.iter_decoded_messages(topics=[topic]):
        msgs.append((message.log_time, ros))
    msgs.sort(key=lambda x: x[0])

    if schema_name == "foxglove_msgs/msg/CompressedVideo":
        fmt = msgs[0][1].format.lower()
        codec = {"h264": "h264", "h265": "hevc", "hevc": "hevc"}.get(fmt)
        if codec is None:
            raise SystemExit(f"{topic}: unsupported CompressedVideo format {fmt!r}")
        blob = b"".join(bytes(r.data) for _t, r in msgs)
        dec = av.CodecContext.create(codec, "r")
        for pkt in dec.parse(blob):
            for fr in dec.decode(pkt):
                yield fr.to_ndarray(format="rgb24")
        for fr in dec.decode(None):
            yield fr.to_ndarray(format="rgb24")

    elif schema_name == "sensor_msgs/msg/CompressedImage":
        for _t, r in msgs:
            arr = np.frombuffer(bytes(r.data), np.uint8)
            import cv2  # optional; only needed for this branch
            img = cv2.imdecode(arr, cv2.IMREAD_COLOR)  # BGR
            yield img[:, :, ::-1].copy()

    else:  # sensor_msgs/msg/Image
        for _t, r in msgs:
            buf = np.frombuffer(bytes(r.data), np.uint8)
            enc = r.encoding.lower()
            if enc in ("rgb8", "bgr8"):
                img = buf.reshape(r.height, r.width, 3)
                if enc == "bgr8":
                    img = img[:, :, ::-1]
            elif enc in ("mono8", "8uc1"):
                img = np.repeat(buf.reshape(r.height, r.width, 1), 3, axis=2)
            else:
                raise SystemExit(f"{topic}: unsupported Image encoding {enc!r}")
            yield np.ascontiguousarray(img)


def stamp(canvas, text):
    img = Image.fromarray(canvas)
    d = ImageDraw.Draw(img)
    tb = d.textbbox((8, 6), text, font=FONT)
    d.rectangle([tb[0] - 6, tb[1] - 4, tb[2] + 6, tb[3] + 4], fill=(0, 0, 0))
    d.text((8, 6), text, font=FONT, fill=(255, 255, 0))
    return np.asarray(img)


def open_writer(path, w, h, fps, crf, preset):
    c = av.open(str(path), mode="w")
    st = c.add_stream("libx264", rate=fps)
    st.width, st.height, st.pix_fmt = w, h, "yuv420p"
    st.codec_context.time_base = Fraction(1, fps)
    st.options = {"crf": str(crf), "preset": preset, "g": str(fps * 2)}
    return c, st


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag", type=Path)
    ap.add_argument("--out", type=Path, help="output .mp4 (single mode) or output dir (--per-camera)")
    ap.add_argument("--topics", nargs="+", help="explicit camera topics / order (default: auto-detect)")
    ap.add_argument("--per-camera", action="store_true", help="one .mp4 per topic instead of one combined")
    ap.add_argument("--canvas", help="WxH for the combined canvas (default: square of max dimension)")
    ap.add_argument("--fps", type=int, default=25)
    ap.add_argument("--crf", type=int, default=21)
    ap.add_argument("--preset", default="faster")
    ap.add_argument("--no-label", action="store_true", help="do not stamp the camera name/res")
    args = ap.parse_args()

    with open(args.bag, "rb") as f:
        reader = make_reader(f, decoder_factories=[DecoderFactory()])
        summary = reader.get_summary()
        detected = detect_camera_topics(summary)
        by_topic = dict(detected)
        if args.topics:
            topics = [(t, by_topic.get(t)) for t in args.topics]
            missing = [t for t, s in topics if s is None]
            if missing:
                raise SystemExit(f"topic(s) not found or not a camera stream: {missing}")
        else:
            topics = detected
        if not topics:
            raise SystemExit("no camera topics found")
        log("cameras: " + ", ".join(t for t, _ in topics))

        out_dir = (args.out if args.per_camera else None) or args.bag.parent / "cvat_output"
        if args.per_camera:
            out_dir = Path(out_dir); out_dir.mkdir(parents=True, exist_ok=True)
        else:
            out_mp4 = Path(args.out) if args.out else args.bag.parent / "cvat_output" / (args.bag.stem + "_cvat.mp4")
            out_mp4.parent.mkdir(parents=True, exist_ok=True)

        # ---- probe first frame of each topic for size ----
        sizes = {}
        for t, s in topics:
            for fr in frames_from_topic(reader, t, s):
                sizes[t] = (fr.shape[1], fr.shape[0])
                break
        log("sizes: " + ", ".join(f"{t}={w}x{h}" for t, (w, h) in sizes.items()))

        if args.per_camera:
            seg_rows = []
            for t, s in topics:
                w, h = sizes[t]
                if w % 2 or h % 2:
                    w, h = w + (w % 2), h + (h % 2)
                dst = out_dir / (t.strip("/").replace("/", "_") + ".mp4")
                c, st = open_writer(dst, w, h, args.fps, args.crf, args.preset)
                n = 0
                for fr in frames_from_topic(reader, t, s):
                    canvas = np.zeros((h, w, 3), np.uint8)
                    canvas[:fr.shape[0], :fr.shape[1]] = fr
                    if not args.no_label:
                        canvas = stamp(canvas, f"{t.split('/')[-2] if '/' in t else t}  {fr.shape[1]}x{fr.shape[0]}")
                    vf = av.VideoFrame.from_ndarray(canvas, format="rgb24")
                    vf.pts = n
                    for pkt in st.encode(vf):
                        c.mux(pkt)
                    n += 1
                for pkt in st.encode():
                    c.mux(pkt)
                c.close()
                seg_rows.append(dict(camera=t, file=dst.name, frames=n, width=w, height=h))
                log(f"wrote {dst.name}  ({n} frames)")
            _write_csv(out_dir / "frame_segments.csv", seg_rows)
            return

        # ---- combined single mp4 ----
        if args.canvas:
            CW, CH = (int(x) for x in args.canvas.lower().split("x"))
        else:
            m = max(max(wh) for wh in sizes.values())
            CW = CH = m + (m % 2)
        log(f"canvas: {CW}x{CH}")
        c, st = open_writer(out_mp4, CW, CH, args.fps, args.crf, args.preset)

        seg_rows = []
        idx = 0
        t0 = time.time()
        for t, s in topics:
            start = idx
            w = h = None
            for fr in frames_from_topic(reader, t, s):
                h, w = fr.shape[:2]
                canvas = np.zeros((CH, CW, 3), np.uint8)
                y0, x0 = (CH - h) // 2, (CW - w) // 2
                canvas[y0:y0 + h, x0:x0 + w] = fr
                if not args.no_label:
                    canvas = stamp(canvas, f"{t.split('/')[-2] if '/' in t else t}  {w}x{h}")
                vf = av.VideoFrame.from_ndarray(canvas, format="rgb24")
                vf.pts = idx
                for pkt in st.encode(vf):
                    c.mux(pkt)
                idx += 1
                if idx % 200 == 0:
                    log(f"  {idx} frames ({idx/(time.time()-t0):.1f} fps)")
            seg_rows.append(dict(camera=t, start_frame=start, end_frame=idx - 1,
                                 frame_count=idx - start, native_width=w, native_height=h))
            log(f"done {t}: frames {start}..{idx-1}")
        for pkt in st.encode():
            c.mux(pkt)
        c.close()
        _write_csv(out_mp4.parent / "frame_segments.csv", seg_rows)
        log(f"WROTE {out_mp4}  ({out_mp4.stat().st_size/1e6:.1f} MB, {idx} frames, "
            f"{idx/args.fps:.1f}s @ {args.fps}fps)")


def _write_csv(path, rows):
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    log(f"wrote {path.name}")


if __name__ == "__main__":
    main()
