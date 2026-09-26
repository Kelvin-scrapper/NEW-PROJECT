"""
verify_supervisely_zip.py - check Supervisely Point Cloud Episodes zips built by
mcap_to_supervisely_pce.py before uploading them.

For each zip it checks:
  * zip integrity        every member's CRC (catches a truncated / half-copied zip)
  * naming               one top folder "<bag>_pce", one episode folder "<bag>"
  * frame counts         .pcd files == frame_pointcloud_map entries == annotation framesCount
  * camera images        same number of .jpg per frame folder (5 for our bags)
  * calibration          every .jpg.json sidecar has intrinsicMatrix (3x3) + extrinsicMatrix (3x4)

Exit code 0 when every zip passes, 1 otherwise.

Examples
--------
    py verify_supervisely_zip.py ..\\supervisely_batch_2026-09-26            # every */*_pce.zip under it
    py verify_supervisely_zip.py some_bag_pce.zip other_bag_pce.zip
    py verify_supervisely_zip.py ..\\supervisely_batch_2026-09-26 --quick    # skip CRC pass (fast)
"""
import argparse
import json
import sys
import zipfile
from collections import Counter
from pathlib import Path


def verify_zip(path, quick=False):
    """-> (ok: bool, info: dict, problems: list[str])"""
    path = Path(path)
    problems = []
    info = {"zip": path.name, "mb": path.stat().st_size / 1e6}
    try:
        z = zipfile.ZipFile(path)
    except zipfile.BadZipFile as e:
        return False, info, [f"not a readable zip: {e}"]

    with z:
        if not quick:
            bad = z.testzip()
            if bad is not None:
                problems.append(f"CRC error in {bad}")

        names = z.namelist()
        tops = sorted({n.split("/")[0] for n in names})
        info["project"] = tops[0] if len(tops) == 1 else tops
        if len(tops) != 1:
            problems.append(f"expected one top-level project folder, found {tops}")
        else:
            expected = path.stem if path.stem.endswith("_pce") else None
            if expected and tops[0] != expected:
                problems.append(f"project folder {tops[0]!r} != zip name {expected!r}")

        for req in ("meta.json", "key_id_map.json"):
            if not any(n.count("/") == 1 and n.endswith("/" + req) for n in names):
                problems.append(f"missing <project>/{req}")

        episodes = sorted({n.split("/")[1] for n in names if n.count("/") >= 2})
        info["episode"] = episodes[0] if len(episodes) == 1 else episodes
        if len(episodes) != 1:
            problems.append(f"expected one episode folder, found {episodes}")

        pcds = [n for n in names if n.endswith(".pcd")]
        jpgs = [n for n in names if n.endswith(".jpg")]
        sidecars = [n for n in names if n.endswith(".jpg.json")]
        info.update(frames=len(pcds), images=len(jpgs))

        fmap_name = next((n for n in names if n.endswith("/frame_pointcloud_map.json")), None)
        ann_name = next((n for n in names if n.endswith("/annotation.json")), None)
        if not fmap_name or not ann_name:
            problems.append("missing frame_pointcloud_map.json or annotation.json")
        else:
            fmap = json.loads(z.read(fmap_name))
            ann = json.loads(z.read(ann_name))
            if not (len(fmap) == len(pcds) == ann.get("framesCount")):
                problems.append(f"frame count mismatch: pcd={len(pcds)} map={len(fmap)} "
                                f"framesCount={ann.get('framesCount')}")
        if not pcds:
            problems.append("no point clouds")

        per_frame = Counter(n.split("/")[-2] for n in jpgs)
        cams = sorted(set(per_frame.values()))
        info["cams_per_frame"] = cams[0] if len(cams) == 1 else cams
        if jpgs and len(cams) != 1:
            problems.append(f"uneven camera images per frame: {dict(Counter(per_frame.values()))}")
        if jpgs and len(per_frame) != len(pcds):
            problems.append(f"{len(per_frame)} image folders for {len(pcds)} frames")

        calibrated = 0
        for n in sidecars:
            sd = json.loads(z.read(n)).get("meta", {}).get("sensorsData", {})
            if len(sd.get("intrinsicMatrix", [])) == 9 and len(sd.get("extrinsicMatrix", [])) == 12:
                calibrated += 1
        info["calibrated"] = f"{calibrated}/{len(jpgs)}"
        if len(sidecars) != len(jpgs):
            problems.append(f"{len(jpgs)} images but {len(sidecars)} .jpg.json sidecars")
        if calibrated != len(jpgs):
            problems.append(f"only {calibrated}/{len(jpgs)} images carry calibration")

    return not problems, info, problems


def collect(targets):
    zips = []
    for t in map(Path, targets):
        if t.is_dir():
            zips += sorted(t.glob("*/*_pce.zip")) + sorted(t.glob("*_pce.zip"))
        else:
            zips.append(t)
    return zips


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("targets", nargs="+", help="zip files and/or batch folders")
    ap.add_argument("--quick", action="store_true", help="skip the CRC pass over every member")
    args = ap.parse_args()

    zips = collect(args.targets)
    if not zips:
        sys.exit("no *_pce.zip found")
    all_ok = True
    print(f"{'status':6}  {'frames':>6}  {'images':>6}  {'cams':>4}  {'calibrated':>11}  {'MB':>7}  zip")
    for zp in zips:
        ok, info, problems = verify_zip(zp, args.quick)
        all_ok &= ok
        print(f"{'OK' if ok else 'FAIL':6}  {info.get('frames', '-'):>6}  {info.get('images', '-'):>6}  "
              f"{str(info.get('cams_per_frame', '-')):>4}  {info.get('calibrated', '-'):>11}  "
              f"{info['mb']:>7.1f}  {zp}")
        for p in problems:
            print(f"        ! {p}")
    sys.exit(0 if all_ok else 1)


if __name__ == "__main__":
    main()
