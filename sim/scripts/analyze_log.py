#!/usr/bin/env python3
"""
Extract the flown track from a PX4 flight log (.ulg) and save it as CSV.

Usage:
    python3 sim/scripts/analyze_log.py                       # newest SITL log
    python3 sim/scripts/analyze_log.py path/to/log.ulg --stage stage1

Output: results/<stage>/track.csv  (t_s, east_m, north_m, alt_m, vx, vy)
Needs:  pip3 install --user pyulog
Note:   PX4 local frame is NED: x = North, y = East, z = Down.
"""
import argparse, csv, glob, os, sys

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def newest_log(px4):
    logs = glob.glob(os.path.join(px4, "build/px4_sitl_default/rootfs/log/*/*.ulg"))
    return max(logs, key=os.path.getmtime) if logs else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log", nargs="?")
    ap.add_argument("--px4", default=os.path.expanduser("~/PX4-Autopilot"))
    ap.add_argument("--stage", default="stage1")
    ap.add_argument("--every", type=int, default=5, help="keep every Nth sample")
    a = ap.parse_args()

    try:
        from pyulog import ULog
    except ImportError:
        sys.exit("pyulog missing: pip3 install --user pyulog")

    log = a.log or newest_log(a.px4)
    if not log:
        sys.exit("no .ulg log found")
    print("log:", log)

    u = ULog(log, ["vehicle_local_position"])
    lp = u.data_list[0].data
    out_dir = os.path.join(REPO, "results", a.stage)
    os.makedirs(out_dir, exist_ok=True)
    out = os.path.join(out_dir, "track.csv")
    with open(out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "east_m", "north_m", "alt_m", "vx", "vy"])
        for i in range(0, len(lp["timestamp"]), a.every):
            w.writerow([round(lp["timestamp"][i] / 1e6, 2), round(lp["y"][i], 2), round(lp["x"][i], 2),
                        round(-lp["z"][i], 2), round(lp["vx"][i], 2), round(lp["vy"][i], 2)])
    print("saved:", out)


if __name__ == "__main__":
    main()
