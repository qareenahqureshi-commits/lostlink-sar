#!/usr/bin/env python3
"""
Stage 5 Monte Carlo: final Stage 4/5 code, on the original building AND on random generated buildings.

Jobs:
  world "stage1": the Stage 1-4 building, 6 policies x 4 drift levels x 10 drift seeds
  world 0..N-1:   generated buildings (make_building.py), 4 policies x 4 drift levels,
                  drift seeds b, b+100, b+200, ... (--build_seeds; 1 = the original v2 sweep)
  --sensitivity:  the proposed method only, one setting changed at a time on the generated buildings:
                  recognition capture radius gamma in {2, 4} m (default 3) and drift prior r0 in {2, 10} %
                  (default 5); the default case is the main sweep itself
Resumable: finished jobs are read back from runs.jsonl and skipped.

Usage:
    python3 sim/offline/stage5_sweep.py [--buildings 10] [--build_seeds 3] [--sensitivity] [--procs 2] [--out ...]
"""
import argparse, json, os, sys, time
from multiprocessing import Pool

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, HERE)
import stage4_bench as B   # noqa: E402

DRIFTS = (2, 5, 10, 20)
POL_STAGE1 = ("rtl_link", "stage3", "map", "map_reloc", "stage4")
POL_BUILD = ("rtl_link", "stage3", "map_reloc", "stage4")


SENS_CAPTURE = (2.0, 4.0)
SENS_PRIOR = (0.02, 0.10)


def jobs(n_build, seeds_stage1=10, build_seeds=1, sensitivity=False):
    J = [("stage1", p, d, s, 0.05, 3.0) for p in POL_STAGE1 for d in DRIFTS for s in range(seeds_stage1)]
    J += [("stage1", "stage4", d, s, 0.02, 3.0) for d in DRIFTS for s in range(seeds_stage1)]
    seeds = lambda b: [b + 100 * k for k in range(build_seeds)]                   # noqa: E731
    J += [(b, p, d, s, 0.05, 3.0) for b in range(n_build) for p in POL_BUILD for d in DRIFTS for s in seeds(b)]
    if sensitivity:
        J += [(b, "stage4", d, s, 0.05, g) for g in SENS_CAPTURE for b in range(n_build) for d in DRIFTS for s in seeds(b)]
        J += [(b, "stage4", d, s, rh, 3.0) for rh in SENS_PRIOR for b in range(n_build) for d in DRIFTS for s in seeds(b)]
    return J


def key(r):
    return (str(r["world"] if r["world"] is not None else "stage1"), r["policy"], float(r["drift_pct"]), int(r["seed"]),
            float(r["r_hat"]), float(r.get("capture_m", 3.0)))


def _job(j):
    world, pol, d, s, rh, g = j
    t0 = time.time()
    r = B.run(pol, d, s, r_hat=rh, capture=g, world=None if world == "stage1" else str(world), wall_limit_s=600)
    r["world"] = "stage1" if world == "stage1" else str(world)
    r["wall_s"] = round(time.time() - t0, 1)
    return r


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--buildings", type=int, default=10)
    ap.add_argument("--build_seeds", type=int, default=1)
    ap.add_argument("--sensitivity", action="store_true")
    ap.add_argument("--procs", type=int, default=os.cpu_count())
    ap.add_argument("--out", default=os.path.join(REPO, "results", "stage5", "offline"))
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    path = os.path.join(a.out, "runs.jsonl")
    done = set()
    if os.path.exists(path):
        for line in open(path):
            try:
                done.add(key(json.loads(line)))
            except Exception:
                pass
    todo = [j for j in jobs(a.buildings, build_seeds=a.build_seeds, sensitivity=a.sensitivity)
            if (str(j[0]), j[1], float(j[2]), int(j[3]), float(j[4]), float(j[5])) not in done]
    # main-sweep seeds first (quick policies first), then the sensitivity runs
    todo.sort(key=lambda j: (j[5] != 3.0 or (j[0] != "stage1" and j[4] != 0.05), j[1] in ("stage3", "map", "map_reloc"), j[0] != "stage1"))
    print(f"{len(todo)} runs to do ({len(done)} done)", flush=True)
    with Pool(a.procs) as pool, open(path, "a") as f:
        for r in pool.imap_unordered(_job, todo):
            f.write(json.dumps({k: v for k, v in r.items() if not k.startswith("_")}) + "\n"); f.flush()
            print(r["world"], r["policy"], r["drift_pct"], r["seed"], r["r_hat"], r["capture_m"], "wp", r["waypoints_reached"],
                  "rooms", r.get("rooms_searched"), "ok" if r["returned_ok"] else "FAIL", r["true_landing_error_m"],
                  "hits", r["collisions"], f"{r['wall_s']}s", flush=True)


if __name__ == "__main__":
    main()
