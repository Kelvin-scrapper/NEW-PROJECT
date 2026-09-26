"""
supervisely_batch.py - build full, calibrated Supervisely Point Cloud Episodes
zips for a whole set of rosbag2 .mcap recordings, one zip per recording, into a
dated batch folder, then verify each one.

    <project root>/supervisely_batch_<YYYY-MM-DD>/
        <bag>/<bag>_pce.zip          the upload
        <bag>/<bag>_build.log        full log of mcap_to_supervisely_pce.py
        batch_manifest.csv           one row per input bag (built / skipped / failed)

What it does per run (see logic.md for the why):
  1. collects the .mcap files (files and/or folders, non-recursive)
  2. finds byte-identical copies (size + SHA-256) - e.g. "bag (1).mcap" from a
     second download - and builds each recording only once
  3. reads each bag's summary to pick its camera topics automatically
     (foxglove CompressedVideo / sensor_msgs CompressedImage), ordered
     main, left side, right side, left rear, right rear
  4. runs mcap_to_supervisely_pce.py on the whole recording (no --count slice),
     calibration on, photo context at --img-maxdim (default 640 px)
  5. verifies the zip (verify_supervisely_zip.py) and records the result

Bags are built one at a time, smallest first. A bag whose zip already exists
and passes a quick check is skipped (use --force to rebuild).

Examples
--------
    py supervisely_batch.py ..\\2609                       # every .mcap in the folder
    py supervisely_batch.py ..\\2609 ..\\extra.mcap --dry-run
    py supervisely_batch.py ..\\2609 --out-root ..\\supervisely_batch_2026-10-01
    py supervisely_batch.py ..\\2609 --img-maxdim 1280     # sharper photo context
"""
import argparse
import csv
import datetime
import hashlib
import subprocess
import sys
import time
from pathlib import Path

from mcap.reader import make_reader

from verify_supervisely_zip import verify_zip

HERE = Path(__file__).resolve().parent
PCE_SCRIPT = HERE / "mcap_to_supervisely_pce.py"
CAMERA_SCHEMAS = ("foxglove_msgs/msg/CompressedVideo", "sensor_msgs/msg/CompressedImage")
# camera order inside every frame folder: 0_main, 1_mast_left_side, ... (stable across bags)
CAMERA_ORDER = ("main", "leftside", "rightside", "leftrear", "rightrear")


def log(m):
    print(f"[{time.strftime('%H:%M:%S')}] {m}", flush=True)


def sha256(path, chunk=8 << 20):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        while b := f.read(chunk):
            h.update(b)
    return h.hexdigest()


def camera_topics(bag):
    with open(bag, "rb") as f:
        summary = make_reader(f).get_summary()
    schemas = {s.id: s.name for s in summary.schemas.values()}
    topics = [ch.topic for ch in summary.channels.values()
              if schemas.get(ch.schema_id) in CAMERA_SCHEMAS]

    def rank(topic):
        key = topic.lower().replace("_", "").replace("mast", "")
        return next((i for i, k in enumerate(CAMERA_ORDER) if k in key), len(CAMERA_ORDER)), topic
    return sorted(set(topics), key=rank)


def collect_bags(inputs):
    bags = []
    for p in map(Path, inputs):
        bags += sorted(p.glob("*.mcap")) if p.is_dir() else [p]
    missing = [b for b in bags if not b.is_file()]
    if missing:
        sys.exit(f"not found: {', '.join(map(str, missing))}")
    return list(dict.fromkeys(b.resolve() for b in bags))


def find_duplicates(bags):
    """-> {duplicate_path: original_path}. The shortest name wins, so
    'x.mcap' is kept over 'x (1).mcap'."""
    by_size = {}
    for b in bags:
        by_size.setdefault(b.stat().st_size, []).append(b)
    dup_of = {}
    for group in by_size.values():
        if len(group) < 2:
            continue
        by_hash = {}
        for b in group:
            log(f"hashing {b.name} (same size as another input)")
            by_hash.setdefault(sha256(b), []).append(b)
        for same in by_hash.values():
            keep, *rest = sorted(same, key=lambda b: (len(b.name), b.name))
            dup_of.update({r: keep for r in rest})
    return dup_of


def build_one(bag, out_dir, cams, img_maxdim):
    out_dir.mkdir(parents=True, exist_ok=True)
    zip_path = out_dir / f"{bag.stem}_pce.zip"
    log_path = out_dir / f"{bag.stem}_build.log"
    cmd = [sys.executable, str(PCE_SCRIPT), str(bag), "--out", str(zip_path),
           "--episode", bag.stem, "--img-maxdim", str(img_maxdim)]
    if cams:
        cmd += ["--context-topics", *cams]
    with open(log_path, "w", encoding="utf-8") as lf:
        rc = subprocess.run(cmd, cwd=HERE, stdout=lf, stderr=subprocess.STDOUT).returncode
    return rc, zip_path, log_path


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("inputs", nargs="+", help=".mcap files and/or folders containing them")
    ap.add_argument("--out-root", type=Path,
                    help="batch folder (default: <project root>/supervisely_batch_<today>)")
    ap.add_argument("--img-maxdim", type=int, default=640, help="photo-context longest side, px")
    ap.add_argument("--force", action="store_true", help="rebuild even if a good zip already exists")
    ap.add_argument("--keep-duplicates", action="store_true",
                    help="also build byte-identical copies under their own names")
    ap.add_argument("--dry-run", action="store_true", help="show the plan, build nothing")
    args = ap.parse_args()

    out_root = args.out_root or HERE.parent / f"supervisely_batch_{datetime.date.today().isoformat()}"
    bags = sorted(collect_bags(args.inputs), key=lambda b: b.stat().st_size)
    dup_of = {} if args.keep_duplicates else find_duplicates(bags)

    log(f"{len(bags)} bag(s) -> {out_root}")
    plan = []
    for b in bags:
        if b in dup_of:
            log(f"  skip  {b.name}  (identical to {dup_of[b].name})")
            plan.append((b, "duplicate", []))
            continue
        cams = camera_topics(b)
        exists = (out_root / b.stem / f"{b.stem}_pce.zip").exists() and not args.force
        log(f"  {'have ' if exists else 'build'} {b.name}  ({b.stat().st_size / 1e6:.0f} MB, "
            f"{len(cams)} cameras){'  - zip exists, will re-check and skip' if exists else ''}")
        plan.append((b, "build", cams))
    if args.dry_run:
        return

    rows, failures = [], 0
    for b, action, cams in plan:
        row = {"bag": b.name, "action": action, "duplicate_of": "", "zip": "", "frames": "",
               "images": "", "calibrated": "", "mb": "", "status": ""}
        if action == "duplicate":
            row.update(duplicate_of=dup_of[b].name, status="skipped")
            rows.append(row)
            continue

        out_dir = out_root / b.stem
        zip_path = out_dir / f"{b.stem}_pce.zip"
        if zip_path.exists() and not args.force and verify_zip(zip_path, quick=True)[0]:
            log(f"{b.stem}: already built and valid - skipping (--force to rebuild)")
            ok, info, problems = verify_zip(zip_path, quick=True)
            row["action"] = "existing"
        else:
            if not cams:
                log(f"{b.stem}: WARNING no camera topics found - building point clouds only")
            log(f"{b.stem}: building ...")
            t0 = time.time()
            rc, zip_path, log_path = build_one(b, out_dir, cams, args.img_maxdim)
            if rc != 0 or not zip_path.exists():
                tail = log_path.read_text(encoding="utf-8", errors="replace").strip().splitlines()[-1:]
                log(f"{b.stem}: FAILED rc={rc}  {tail[0] if tail else ''}  (see {log_path})")
                row.update(status=f"failed rc={rc}")
                rows.append(row)
                failures += 1
                continue
            log(f"{b.stem}: built in {(time.time() - t0) / 60:.1f} min, verifying ...")
            ok, info, problems = verify_zip(zip_path)

        row.update(zip=str(zip_path.relative_to(out_root)), frames=info.get("frames"),
                   images=info.get("images"), calibrated=info.get("calibrated"),
                   mb=f"{info['mb']:.1f}", status="ok" if ok else "; ".join(problems))
        rows.append(row)
        failures += not ok
        log(f"{b.stem}: {'OK' if ok else 'PROBLEMS'}  frames={info.get('frames')} "
            f"calibrated={info.get('calibrated')} {info['mb']:.0f} MB"
            + ("" if ok else "  ! " + "; ".join(problems)))

    out_root.mkdir(parents=True, exist_ok=True)
    manifest = out_root / "batch_manifest.csv"
    with open(manifest, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    log(f"manifest: {manifest}")
    log("ALL DONE" if not failures else f"DONE with {failures} problem(s)")
    sys.exit(1 if failures else 0)


if __name__ == "__main__":
    main()
