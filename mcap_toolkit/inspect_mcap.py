"""
inspect_mcap.py - print what is inside a rosbag2 .mcap file.

Usage:
    py inspect_mcap.py BAG.mcap
    py inspect_mcap.py BAG.mcap --first-message /some/topic
    py inspect_mcap.py BAG.mcap --tf            # dump /tf_static transforms
    py inspect_mcap.py BAG.mcap --schema foxglove_msgs/msg/CompressedVideo

No ROS install required.
"""
import argparse
from pathlib import Path

from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory

PF_TYPE = {1: "int8", 2: "uint8", 3: "int16", 4: "uint16",
           5: "int32", 6: "uint32", 7: "float32", 8: "float64"}


def human_rate(count, start_ns, end_ns):
    dur = (end_ns - start_ns) / 1e9
    return f"{count/dur:.2f} Hz" if dur > 0 else "?"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag", type=Path)
    ap.add_argument("--first-message", metavar="TOPIC",
                    help="dump structural details of the first message on TOPIC")
    ap.add_argument("--schema", metavar="NAME",
                    help="print the full schema text whose name contains NAME")
    ap.add_argument("--tf", action="store_true",
                    help="print every transform found in /tf_static")
    args = ap.parse_args()

    with open(args.bag, "rb") as f:
        reader = make_reader(f, decoder_factories=[DecoderFactory()])
        summary = reader.get_summary()
        header = reader.get_header()
        stats = summary.statistics
        schemas = {s.id: s for s in summary.schemas.values()}
        ch_counts = stats.channel_message_counts if stats else {}

        print(f"file      : {args.bag}")
        print(f"size      : {args.bag.stat().st_size/1e6:.1f} MB")
        print(f"profile   : {header.profile!r}   library: {header.library!r}")
        if stats:
            dur = (stats.message_end_time - stats.message_start_time) / 1e9
            print(f"messages  : {stats.message_count}   duration: {dur:.2f} s")
            print(f"time span : {stats.message_start_time} .. {stats.message_end_time} (ns)")
        print()
        print(f"{'TOPIC':<52} {'MSGS':>7}  {'RATE':>10}  TYPE")
        print("-" * 110)
        for ch in sorted(summary.channels.values(), key=lambda c: c.topic):
            sch = schemas.get(ch.schema_id)
            n = ch_counts.get(ch.id, 0)
            rate = human_rate(n, stats.message_start_time, stats.message_end_time) if stats else "?"
            print(f"{ch.topic:<52} {n:>7}  {rate:>10}  {sch.name if sch else '?'} [{ch.message_encoding}]")

        if args.schema:
            for s in schemas.values():
                if args.schema.lower() in s.name.lower():
                    print(f"\n=== SCHEMA {s.name}  (encoding: {s.encoding}) ===")
                    print(s.data.decode("utf-8", "replace"))

        if args.tf:
            print("\n=== /tf_static ===")
            for _s, ch, _m, ros in reader.iter_decoded_messages(topics=["/tf_static"]):
                for tr in ros.transforms:
                    t, q = tr.transform.translation, tr.transform.rotation
                    print(f"  {tr.header.frame_id:<22} -> {tr.child_frame_id:<26} "
                          f"xyz=({t.x:+.4f},{t.y:+.4f},{t.z:+.4f}) "
                          f"quat=({q.x:+.5f},{q.y:+.5f},{q.z:+.5f},{q.w:+.5f})")
                break

        if args.first_message:
            tp = args.first_message
            print(f"\n=== first message on {tp} ===")
            for _s, ch, msg, ros in reader.iter_decoded_messages(topics=[tp]):
                print(f"  log_time : {msg.log_time}")
                if hasattr(ros, "header"):
                    h = ros.header
                    print(f"  stamp    : {h.stamp.sec}.{h.stamp.nanosec:09d}   frame_id: {h.frame_id!r}")
                if hasattr(ros, "fields"):  # PointCloud2
                    print(f"  width={ros.width} height={ros.height} point_step={ros.point_step} "
                          f"row_step={ros.row_step} is_dense={ros.is_dense} is_bigendian={ros.is_bigendian}")
                    for ff in ros.fields:
                        print(f"    field {ff.name:<14} offset={ff.offset:<3} "
                              f"type={PF_TYPE.get(ff.datatype, ff.datatype)} count={ff.count}")
                if hasattr(ros, "format"):  # CompressedVideo / CompressedImage
                    data = bytes(ros.data)
                    print(f"  format   : {ros.format!r}   data bytes: {len(data)}")
                    print(f"  head     : {' '.join(f'{b:02x}' for b in data[:16])}")
                if hasattr(ros, "encoding") and hasattr(ros, "step"):  # raw Image
                    print(f"  {ros.width}x{ros.height} encoding={ros.encoding!r} step={ros.step}")
                break


if __name__ == "__main__":
    main()
