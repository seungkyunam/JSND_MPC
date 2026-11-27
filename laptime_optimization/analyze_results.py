#!/usr/bin/env python3
"""Create reproducible tables and figures from saved lap-time experiments."""
from __future__ import annotations

import csv
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
OUT = Path(__file__).resolve().parent / "analysis"
OUT.mkdir(exist_ok=True)
PP_TRACE = ROOT / "test0_mpc_improvement/final_validation/test0_baseline_ideal_pp_clearance035/trace.csv"
BASE_TRACE = ROOT / "test0_mpc_improvement/final_validation/test0_baseline_ideal_improved1s_qpos20/trace.csv"
OPT_TRACE = Path(__file__).resolve().parent / "final_11lap/test0_obstacles_window160/trace.csv"
MAP_JSON = ROOT / "stack_master/maps/test0/global_waypoints.json"


def rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="") as handle:
        return list(csv.DictReader(handle))


def summary(path: Path) -> dict:
    return json.loads(path.read_text())


def aligned(trace: Path, edges: np.ndarray) -> dict[str, np.ndarray]:
    data = [r for r in rows(trace) if int(float(r["lap"])) > 1]
    laps = max(1, len({int(float(r["lap"])) for r in data}))
    s = np.array([float(r["s"]) for r in data])
    idx = np.clip(np.digitize(s, edges) - 1, 0, len(edges) - 2)
    result: dict[str, np.ndarray] = {}
    for key in ("speed", "accel", "command_speed", "solver_failure"):
        values = np.array([float(r.get(key, 0) or 0) for r in data])
        result[key] = np.array([
            values[idx == i].mean() if np.any(idx == i) else np.nan
            for i in range(len(edges) - 1)
        ])
    counts = np.bincount(idx, minlength=len(edges) - 1)
    result["time"] = counts * 0.02 / laps
    return result


def loss_analysis() -> None:
    waypoint = json.loads(MAP_JSON.read_text())["global_traj_wpnts_iqp"]["wpnts"]
    length = float(waypoint[-1]["s_m"])
    edges = np.arange(0, length + 0.5, 0.5)
    center = (edges[:-1] + edges[1:]) / 2
    pp, mpc = aligned(PP_TRACE, edges), aligned(BASE_TRACE, edges)
    optimized = aligned(OPT_TRACE, edges) if OPT_TRACE.exists() else None
    wp_s = np.array([p["s_m"] for p in waypoint])
    curvature = np.interp(center, wp_s, np.abs([p["kappa_radpm"] for p in waypoint]))
    raceline_cap = np.interp(center, wp_s, [p["vx_mps"] for p in waypoint])
    loss = mpc["time"] - pp["time"]

    with (OUT / "s_aligned_loss.csv").open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(("s_center_m", "pp_speed_mps", "mpc_speed_mps", "pp_accel_mps2",
                         "mpc_accel_mps2", "curvature_1pm", "raceline_cap_mps",
                         "mpc_command_mps", "mpc_failure_fraction", "time_loss_s"))
        for i, pos in enumerate(center):
            writer.writerow((pos, pp["speed"][i], mpc["speed"][i], pp["accel"][i],
                             mpc["accel"][i], curvature[i], raceline_cap[i],
                             mpc["command_speed"][i], mpc["solver_failure"][i], loss[i]))

    fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
    axes[0].plot(center, pp["speed"], label="PP measured")
    axes[0].plot(center, mpc["speed"], label="stable 1 s MPC measured")
    if optimized is not None:
        axes[0].plot(center, optimized["speed"], label="160-point candidate measured")
    axes[0].plot(center, mpc["command_speed"], "--", label="MPC command")
    axes[0].plot(center, raceline_cap, ":", label="global raceline cap")
    axes[0].set_ylabel("speed (m/s)"); axes[0].legend(ncol=2)
    axes[1].plot(center, pp["accel"], label="PP")
    axes[1].plot(center, mpc["accel"], label="MPC")
    axes[1].axhline(0, color="black", lw=.5); axes[1].set_ylabel("accel (m/s²)"); axes[1].legend()
    axes[2].plot(center, curvature, color="tab:purple")
    axes[2].set_ylabel("|curvature| (1/m)")
    axes[3].bar(center, loss, width=.45, label="MPC − PP time")
    axes[3].plot(center, mpc["solver_failure"] * .2, color="tab:red", label="failure fraction × 0.2")
    axes[3].axvspan(22, 30, color="orange", alpha=.15, label="dominant loss region")
    axes[3].set_ylabel("time loss (s/0.5 m)"); axes[3].set_xlabel("track position s (m)")
    axes[3].legend()
    fig.suptitle(f"Saved-log s-aligned diagnosis (total MPC loss {np.nansum(loss):.3f} s/lap)")
    fig.tight_layout(); fig.savefig(OUT / "s_aligned_speed_loss.png", dpi=180); plt.close(fig)


def collect_summaries() -> None:
    records: list[dict] = []
    groups = {
        "terminal": Path(__file__).parent / "screen_terminal",
        "objective": Path(__file__).parent / "screen_objective",
        "window": Path(__file__).parent / "screen_window",
        "speed_cap": Path(__file__).parent / "screen_speed_cap",
        "crossmap": Path(__file__).parent / "crossmap_3lap",
        "final_11lap": Path(__file__).parent / "final_11lap",
        "benchmark_11lap": Path(__file__).parent / "benchmarks_11lap",
    }
    for group, directory in groups.items():
        if not directory.exists():
            continue
        for path in sorted(directory.glob("*/summary.json")):
            s = summary(path)
            records.append({
                "group": group, "candidate": path.parent.name, "map": s.get("map"),
                "completed_laps": s.get("completed_laps"), "mean_lap_s": s.get("mean_lap_s"),
                "best_lap_s": s.get("best_lap_s"), "mean_speed_mps": s.get("mean_speed_mps"),
                "max_speed_mps": s.get("max_speed_mps"), "rmse_m": s.get("tracking_rmse_m"),
                "failure_pct": s.get("solver_failure_pct"), "cycle_overrun_pct": s.get("cycle_overrun_pct"),
                "collision_events": s.get("collision_events", 0), "stop_events": s.get("stop_events", 0),
                "terminal_stop": s.get("terminal_stop", False),
                "physical_violation_pct": s.get("physical_violation_pct", s.get("combined_limit_violation_pct")),
            })
    columns = list(records[0])
    with (OUT / "experiment_summary.csv").open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=columns); writer.writeheader(); writer.writerows(records)

    caps = [r for r in records if r["group"] == "speed_cap" and r["candidate"].startswith("actual_plant")]
    caps.sort(key=lambda r: float(r["candidate"].removeprefix("actual_plant_v")))
    if caps:
        x = [float(r["candidate"].removeprefix("actual_plant_v")) for r in caps]
        fig, ax1 = plt.subplots(figsize=(8, 4.5)); ax2 = ax1.twinx()
        ax1.plot(x, [r["mean_lap_s"] for r in caps], "o-", label="mean lap")
        ax2.plot(x, [r["max_speed_mps"] for r in caps], "s--", color="tab:red", label="actual max speed")
        ax1.set_xlabel("MPC speed ceiling (m/s)"); ax1.set_ylabel("mean lap (s)")
        ax2.set_ylabel("actual max speed (m/s)"); ax2.axhline(5.039, color="gray", ls=":", label="plant cap")
        lines = ax1.lines + ax2.lines; ax1.legend(lines, [l.get_label() for l in lines], loc="best")
        ax1.set_title("Speed-ceiling screen with unchanged physical plant")
        fig.tight_layout(); fig.savefig(OUT / "speed_ceiling_screen.png", dpi=180); plt.close(fig)

    comparison = [
        ("PP", ROOT / "test0_mpc_improvement/final_validation/test0_baseline_ideal_pp_clearance035/summary.json"),
        ("stable qpos20", ROOT / "test0_mpc_improvement/final_validation/test0_baseline_ideal_improved1s_qpos20/summary.json"),
        ("160-point", Path(__file__).parent / "final_11lap/test0_obstacles_window160/summary.json"),
        ("terminal 1.5", Path(__file__).parent / "final_11lap/test0_obstacles_terminal15_correct/summary.json"),
    ]
    comparison = [(label, summary(path)) for label, path in comparison if path.exists()]
    if comparison:
        labels = [x[0] for x in comparison]
        means = [x[1]["mean_lap_s"] for x in comparison]
        stds = [x[1].get("lap_std_s", 0) for x in comparison]
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4.5))
        ax1.bar(labels, means, yerr=stds, capsize=4)
        ax1.set_ylabel("mean lap ± std (s)"); ax1.tick_params(axis="x", rotation=20)
        ax1.set_title("test0, two static obstacles, 11 laps")
        ax2.bar(labels, [x[1]["mean_speed_mps"] for x in comparison], label="mean speed")
        ax2.scatter(labels, [x[1]["max_speed_mps"] for x in comparison], color="tab:red", label="max speed")
        ax2.set_ylabel("speed (m/s)"); ax2.tick_params(axis="x", rotation=20); ax2.legend()
        fig.tight_layout(); fig.savefig(OUT / "final_candidate_comparison.png", dpi=180); plt.close(fig)


if __name__ == "__main__":
    loss_analysis()
    collect_summaries()
