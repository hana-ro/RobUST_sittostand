"""Sagittal trajectory tracking during assist-as-needed trials.

Pick a CSV folder. The file name picks the segment (Pelvis -> CoM, Trunk -> EE).
Each trial (rows where the segment's tunnel is active) gets the actual path against
the reference path and tunnel walls, and the distance from the path over time.

The reference path is read from the log's *_Path_* columns. Older logs lack them,
so the path is rebuilt from the trajectory CSV: its origin is the seated target held
during the countdown, and the leg length is fitted so the path passes through the
logged nearest points (*_Goal_P*). The fit error is shown as a check.

Run: python trajectory_tracking.py
"""

from pathlib import Path
import tkinter as tk
from tkinter import filedialog, messagebox

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from plot_output_wrench import segment_for

INNER_M, OUTER_M = 0.03, 0.06  # AssistAsNeededController tunnel walls
# RobUSTDescription.FloorZ: mean force-plate corner height, -0.01 m correction
FLOOR_Z = np.mean([-1.68039029844029, -1.66494216707722,
                   -1.72241844826073, -1.70847242495829]) - 0.01


def xyz(data, name):
    return data[[f"{name}_P{a}" for a in "xyz"]].to_numpy(dtype=float)


def distance_to_path(points, path):
    """Sagittal (x, z) distance from each point to the path polyline [m]."""
    p, a = points[:, None, [0, 2]], path[:-1, [0, 2]]
    ab = path[1:, [0, 2]] - a
    t = np.clip(((p - a) * ab).sum(-1) / np.maximum((ab * ab).sum(-1), 1e-18), 0, 1)
    return np.linalg.norm(p - (a + t[..., None] * ab), axis=-1).min(axis=1)


def reference_path(data, prefix, segment, start, end):
    """(path, seated position, note) for the trial starting at row `start`."""
    if f"{prefix}_Path_Px" in data.columns:
        rows = data[(data["TrialIndex"] == data["TrialIndex"].iloc[start])
                    & data[f"{prefix}_Path_Px"].notna()]
        seat = "Trial_SeatedCoM" if prefix == "CoM" else "Trial_SeatedEE"
        return (xyz(rows, f"{prefix}_Path"), xyz(rows, seat)[0],
                f"logged path, leg length {rows['Trial_LegLength'].iloc[0]:.3f} m")

    # Rebuild: Robot frame forward is -x, left is -y; x, y scale with leg length, z with rise.
    traj = pd.read_csv(Path(__file__).parent / "Trajectories"
                       / f"SitToWalk_GPR_{segment}_NoVR_normalized.csv")
    disp = np.column_stack([-traj["delta_AP_forward_pct"], -traj["delta_ML_left_pct"],
                            traj["delta_Vertical_up_pct"]]) / 100
    seated = xyz(data, f"{prefix}_Goal")[start - 1]  # countdown target = path origin
    seated_height = data["CoM_Goal_Pz"].iloc[start - 1] - FLOOR_Z
    nearest = xyz(data, f"{prefix}_Goal")[start:end]
    build = lambda L: seated + disp * [L, L, L - seated_height]
    cost = lambda L: np.median(distance_to_path(nearest, build(L)))
    L = min(np.arange(0.5, 2.0, 0.01), key=cost)
    L = min(np.arange(L - 0.01, L + 0.01, 0.0002), key=cost)
    return build(L), seated, f"rebuilt path, fitted leg length {L:.3f} m (fit error {cost(L) * 1000:.2f} mm)"


def plot_trial(name, segment, prefix, data, time, start, end, number):
    path, seated, note = reference_path(data, prefix, segment, start, end)
    actual = xyz(data, f"{prefix}_VirtualFront")[start:end]
    dev_mm = data[f"{prefix}_Deviation"].to_numpy()[start:end] * INNER_M * 1000
    done = np.flatnonzero(data[f"{prefix}_Progress"].to_numpy()[start:end] >= 1)
    t = time[start:end] - time[start]
    outcome = (f"completed in {t[done[0]]:.2f} s" if done.size else
               f"not completed ({data[f'{prefix}_Progress'].iloc[start:end].max():.0%} of path)")

    fig, (ax_path, ax_dev) = plt.subplots(1, 2, figsize=(14, 6), layout="constrained")
    fig.suptitle(f"{name} | {segment} ({prefix}) trial {number} | {outcome} | "
                 f"max distance {dev_mm.max():.0f} mm | "
                 f"beyond inner wall {np.mean(dev_mm > INNER_M * 1000):.0%} of time\n{note}",
                 fontsize=10)

    # Sagittal plane in cm from the seated position: forward = -x, up = +z
    sag = lambda p: ((seated - p)[:, 0] * 100, (p - seated)[:, 2] * 100)
    fwd, up = sag(path)
    tangent = np.gradient(np.column_stack([fwd, up]), axis=0)
    normal = np.column_stack([-tangent[:, 1], tangent[:, 0]])
    normal /= np.linalg.norm(normal, axis=1, keepdims=True)
    centre = np.column_stack([fwd, up])
    for lo, hi, color, alpha in ((0, INNER_M, "#81c784", 0.25), (INNER_M, OUTER_M, "#ffcc80", 0.3)):
        for side in (1, -1):  # each band on both sides of the path, without overlap
            band = np.r_[centre + side * lo * 100 * normal, (centre + side * hi * 100 * normal)[::-1]]
            ax_path.fill(band[:, 0], band[:, 1], color=color, alpha=alpha, lw=0)
    ax_path.plot(fwd, up, color="#2e7d32", label="Reference")
    afwd, aup = sag(actual)
    ax_path.plot(afwd, aup, color="black", label="Actual")
    ax_path.plot(afwd[0], aup[0], "o", color="black", ms=5)   # start
    ax_path.plot(afwd[-1], aup[-1], "s", color="black", ms=5)  # end
    ax_path.set(aspect="equal", xlabel="Forward (cm)", ylabel="Up (cm)",
                title="Sagittal path (green: inside inner wall, orange: inner to outer wall)")
    ax_path.legend()

    top_mm = max(OUTER_M * 1200, dev_mm.max() * 1.1)
    for lo, hi, color, alpha in ((0, INNER_M * 1000, "#81c784", 0.25),
                                 (INNER_M * 1000, OUTER_M * 1000, "#ffcc80", 0.3),
                                 (OUTER_M * 1000, top_mm, "#ef9a9a", 0.25)):
        ax_dev.axhspan(lo, hi, color=color, alpha=alpha)
    ax_dev.plot(t, dev_mm, color="black")
    ax_dev.set(xlabel="Time (s)", ylabel="Distance from path (mm)", ylim=(0, None),
               title="Distance from path")
    for ax in (ax_path, ax_dev):
        ax.grid(True, alpha=0.3)


def analyze(csv_path):
    segment, prefix = segment_for(csv_path)
    data = pd.read_csv(csv_path, encoding="utf-8-sig")
    data.columns = data.columns.str.replace(" ", "", regex=False)
    # Tunnel deviation is >= 0 only while an assist-as-needed trial runs and the body is tracked.
    active = ((data["ControlMode"].astype(str).str.strip() == "ASSIST_AS_NEEDED")
              & (data[f"{prefix}_Deviation"] >= 0)).to_numpy()
    edges = np.diff(np.r_[0, active.astype(int), 0])
    trials = list(zip(np.flatnonzero(edges == 1), np.flatnonzero(edges == -1)))
    if not trials:
        raise ValueError("no assist-as-needed trial")
    time = data["Timestamp"].to_numpy(dtype=float)
    for number, (start, end) in enumerate(trials, 1):
        plot_trial(csv_path.name, segment, prefix, data, time, start, end, number)


def main():
    root = tk.Tk()
    root.withdraw()
    folder = filedialog.askdirectory(title="Select folder containing CSV logs")
    for csv_path in sorted(Path(folder).glob("*.csv")) if folder else []:
        try:
            analyze(csv_path)
        except (ValueError, KeyError, OSError) as exc:
            messagebox.showwarning("CSV skipped", f"{csv_path.name}: {exc}")
    root.destroy()
    plt.show()


if __name__ == "__main__":
    main()
