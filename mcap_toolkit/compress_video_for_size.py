"""
compress_video_for_size.py - shrink an MP4 so it fits under a hard size cap
(e.g. Supervisely's 25 MB) while keeping every camera segment.

  1. keeps every Nth frame            (--stride)
  2. resizes to --size                (area-averaged downscale)
  3. searches for the LOWEST x264 CRF (= best quality) whose output is <= --max-mb

Constant-quality (CRF) encoding, so bits go where the picture needs them. The
decimated, resized frames are cached in RAM, so each search step is one encode.

If frame_segments.csv (from mcap_to_cvat_video.py) sits next to the input, a
matching <out>_segments.csv is written with the NEW frame range of each camera.

Examples
--------
    py compress_video_for_size.py IN.mp4 --out OUT.mp4
    py compress_video_for_size.py IN.mp4 --out OUT.mp4 --max-mb 24 --stride 10 --size 1024x1024
    py compress_video_for_size.py IN.mp4 --out OUT.mp4 --stride 5 --size 768x768   # more frames, smaller
"""
import argparse
import csv
import math
import time
from fractions import Fraction
from pathlib import Path

import av


def log(m):
    print(f"[{time.strftime('%H:%M:%S')}] {m}", flush=True)


def make_scaler(W, H):
    """Return f(frame)->yuv420p frame of size WxH, using the best interpolation PyAV accepts."""
    state = {"interp": None}

    def scale(fr):
        if state["interp"] is None:
            for interp in ("AREA", "BICUBIC", ""):
                try:
                    out = (fr.reformat(width=W, height=H, format="yuv420p", interpolation=interp)
                           if interp else fr.reformat(width=W, height=H, format="yuv420p"))
                    state["interp"] = interp
                    return out
                except (TypeError, ValueError):
                    continue
            raise RuntimeError("PyAV reformat failed for every interpolation mode")
        interp = state["interp"]
        return (fr.reformat(width=W, height=H, format="yuv420p", interpolation=interp)
                if interp else fr.reformat(width=W, height=H, format="yuv420p"))
    return scale


def load_frames(src, stride, W, H):
    inp = av.open(str(src))
    v = inp.streams.video[0]
    v.thread_type = "AUTO"
    src_fps = float(v.average_rate)
    total = v.frames or 0
    scale = make_scaler(W, H)
    frames, idx = [], []
    t0 = time.time()
    for i, fr in enumerate(inp.decode(v)):
        if i % stride == 0:
            frames.append(scale(fr))
            idx.append(i)
        if i and i % 1000 == 0:
            log(f"  decoded {i}/{total or '?'} source frames ({i / (time.time() - t0):.0f} fps), kept {len(frames)}")
    inp.close()
    return frames, idx, src_fps


def encode(frames, path, W, H, fps, crf, preset, gop):
    path = Path(path)
    if path.exists():
        path.unlink()
    tb = Fraction(1) / fps
    out = av.open(str(path), "w", options={"movflags": "faststart"})
    st = out.add_stream("libx264", rate=fps)
    st.width, st.height, st.pix_fmt = W, H, "yuv420p"
    st.codec_context.time_base = tb
    st.options = {"crf": f"{crf:.2f}", "preset": preset, "g": str(gop)}
    for k, fr in enumerate(frames):
        fr.pts = k
        fr.time_base = tb
        for pkt in st.encode(fr):
            out.mux(pkt)
    for pkt in st.encode():
        out.mux(pkt)
    out.close()
    return path.stat().st_size


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("src", type=Path)
    ap.add_argument("--out", type=Path, help="default: <src stem>_small.mp4 next to the input")
    ap.add_argument("--max-mb", type=float, default=24.0,
                    help="hard ceiling in decimal MB (1 MB = 1,000,000 B). 24 fits under both a 25 MB and a 25 MiB cap")
    ap.add_argument("--stride", type=int, default=10, help="keep every Nth source frame")
    ap.add_argument("--size", default="1024x1024", help="output WxH")
    ap.add_argument("--preset", default="slower")
    ap.add_argument("--crf-min", type=float, default=16.0, help="best quality the search may pick")
    ap.add_argument("--crf-max", type=float, default=46.0, help="worst quality the search may pick")
    ap.add_argument("--crf-start", type=float, default=30.0)
    ap.add_argument("--max-iters", type=int, default=8)
    ap.add_argument("--segments-csv", type=Path, help="source frame_segments.csv (default: next to input)")
    args = ap.parse_args()

    W, H = (int(x) for x in args.size.lower().split("x"))
    W += W % 2
    H += H % 2
    out = args.out or args.src.with_name(args.src.stem + "_small.mp4")
    cap = int(args.max_mb * 1_000_000)
    goal = 0.965 * cap          # aim just under the cap; accept anything in [0.92*cap, cap]

    log(f"source {args.src.name}  ({args.src.stat().st_size / 1e6:.1f} MB)")
    log(f"target: every {args.stride}th frame, {W}x{H}, <= {cap / 1e6:.1f} MB ({cap / 2**20:.2f} MiB)")
    frames, idx, src_fps = load_frames(args.src, args.stride, W, H)
    fps = Fraction(src_fps).limit_denominator(1000) / args.stride
    gop = max(2, int(round(float(fps) * 30)))
    log(f"kept {len(frames)} frames -> {float(fps):.3f} fps, {len(frames) / float(fps):.1f}s, GOP {gop}")

    tmp = out.with_suffix(".try.mp4")
    probes = []                 # (crf, size)
    best = None                 # (crf, size) smallest CRF that fits
    lo, hi = args.crf_min, args.crf_max
    crf = args.crf_start
    best_path = out.with_suffix(".best.mp4")

    for it in range(args.max_iters):
        t0 = time.time()
        size = encode(frames, tmp, W, H, fps, crf, args.preset, gop)
        probes.append((crf, size))
        fits = size <= cap
        log(f"  try {it + 1}: crf {crf:5.2f} -> {size / 1e6:6.2f} MB  {'FITS' if fits else 'too big'}  ({time.time() - t0:.0f}s)")
        if fits:
            if best is None or crf < best[0]:
                best = (crf, size)
                if best_path.exists():
                    best_path.unlink()
                tmp.replace(best_path)
            hi = min(hi, crf)
            if size >= 0.92 * cap:
                break
        else:
            lo = max(lo, crf)
        if hi - lo < 0.3:
            break
        # next CRF: interpolate log(size) between the two probes bracketing the goal, else use the ~-11%/CRF rule
        below = [(c, s) for c, s in probes if s <= goal]
        above = [(c, s) for c, s in probes if s > goal]
        if below and above:
            c1, s1 = max(above, key=lambda p: p[0])          # too big, highest CRF
            c2, s2 = min(below, key=lambda p: p[0])          # fits, lowest CRF
            t = (math.log(s1) - math.log(goal)) / max(1e-9, (math.log(s1) - math.log(s2)))
            nxt = c1 + t * (c2 - c1)
        else:
            nxt = crf + 6.0 * math.log2(size / goal)
        crf = min(max(nxt, lo + 0.15), hi - 0.15) if hi > lo + 0.3 else (lo + hi) / 2

    if best is None:
        log(f"no CRF in [{args.crf_min}, {args.crf_max}] fit under {cap / 1e6:.1f} MB - "
            f"lower --size or raise --stride, then re-run")
        raise SystemExit(1)

    if out.exists():
        out.unlink()
    best_path.replace(out)
    if tmp.exists():
        tmp.unlink()
    crf_best, size_best = best
    log(f"WROTE {out}  {size_best / 1e6:.2f} MB ({size_best / 2**20:.2f} MiB)  crf {crf_best:.2f}  "
        f"{len(frames)} frames @ {float(fps):.3f} fps  {W}x{H}")
    log(f"bits/pixel/frame: {size_best * 8 / (len(frames) * W * H):.3f}")
    if crf_best > 38:
        log("WARNING: CRF > 38 - picture will be soft. Consider a smaller --size or larger --stride.")

    # remap the camera segments onto the kept frames
    seg_csv = args.segments_csv or args.src.with_name("frame_segments.csv")
    if seg_csv.exists():
        with open(seg_csv, newline="", encoding="utf-8") as f:
            segs = list(csv.DictReader(f))
        rows = []
        for s in segs:
            a, b = int(s["start_frame"]), int(s["end_frame"])
            ks = [k for k, i in enumerate(idx) if a <= i <= b]
            if ks:
                rows.append({"camera": s["camera"], "start_frame": ks[0], "end_frame": ks[-1],
                             "frame_count": len(ks), "source_start": idx[ks[0]], "source_end": idx[ks[-1]]})
        seg_out = out.with_name(out.stem + "_segments.csv")
        with open(seg_out, "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            w.writeheader()
            w.writerows(rows)
        log(f"wrote {seg_out.name}")
    log("ALL DONE")


if __name__ == "__main__":
    main()
