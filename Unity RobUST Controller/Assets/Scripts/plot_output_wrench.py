"""Pick a CSV folder and plot the commanded (solver) versus measured output
wrench of a body segment for each file.

The segment is chosen from the file name, e.g. "Trunk_3_2026-11-17_15-28-43.csv":
  Pelvis -> CoM_* columns
  Trunk  -> EE_* columns
Files naming neither segment are skipped with a warning.

Error metrics (average and max absolute error, % error and index of the
max-error sample) use only assist-as-needed samples.

Install dependencies: python -m pip install pandas matplotlib
Run: python plot_output_wrench.py
Figures are displayed without saving files.
"""

from pathlib import Path
import tkinter as tk
from tkinter import filedialog, messagebox

import matplotlib.pyplot as plt
from matplotlib.patches import Patch
import numpy as np
import pandas as pd


COMPONENTS = ("Fx", "Fy", "Fz", "Tx", "Ty", "Tz")
# File-name keyword -> CSV column prefix written by DataLogger.cs.
SEGMENTS = {"Pelvis": "CoM", "Trunk": "EE"}
METRIC_MODE = "ASSIST_AS_NEEDED"  # error metrics use only samples in this mode
MODE_STYLES = {
    "ASSIST_AS_NEEDED": ("A: Assist-as-needed", "#81c784"),
    "TRANSPARENT": ("T: Transparent", "#bdbdbd"),
    "IMPEDANCE": ("I: Impedance", "#90caf9"),
    "OFF": ("O: Off", "#ffcc80"),
}


def mode_regions(x, modes):
    """Runs start at the first sample reporting the new mode; last ends at last sample."""
    x = np.asarray(x, dtype=float)
    modes = modes.fillna("UNKNOWN").astype(str).str.strip().str.upper().to_numpy()
    if not np.all(np.isfinite(x)) or np.any(np.diff(x) < 0):
        raise ValueError("Mode shading requires finite, nondecreasing timestamps")
    starts = np.r_[0, np.flatnonzero(modes[1:] != modes[:-1]) + 1]
    return [(x[start], x[end] if end < len(x) else x[-1], modes[start])
            for start, end in zip(starts, np.r_[starts[1:], len(x)])]


def error_metrics(commanded, measured, include):
    """Absolute errors and percentage at the first largest-error sample.

    Only samples where `include` is True count. max_index is the row index in
    the whole CSV. Percentage uses the magnitude of that sample's solver value,
    not the peak solver value or the largest percentage across the samples.
    """
    commanded = np.asarray(commanded, dtype=float)
    measured = np.asarray(measured, dtype=float)
    valid = np.isfinite(commanded) & np.isfinite(measured) & np.asarray(include, dtype=bool)
    count = int(valid.sum())
    if not count:
        return dict(mae=np.nan, mae_pct=np.nan, max_abs=np.nan, max_error_pct=np.nan,
                    max_index=None)
    error = np.abs(measured[valid] - commanded[valid])
    peak = int(np.argmax(error))
    command_magnitude = abs(commanded[valid][peak])
    percentage = (100 * error[peak] / command_magnitude
                  if command_magnitude > 1e-12 else np.nan)
    # Average error as a % of the average |solver| over the same samples. Dividing
    # the averages (not averaging per-sample %) avoids blow-ups where solver ~ 0.
    mean_command = np.mean(np.abs(commanded[valid]))
    mae_pct = 100 * np.mean(error) / mean_command if mean_command > 1e-12 else np.nan
    return dict(mae=np.mean(error), mae_pct=mae_pct, max_abs=error[peak],
                max_error_pct=percentage,
                max_index=int(np.flatnonzero(valid)[peak]))


def format_metric(value):
    return f"{value:.3g}" if np.isfinite(value) else "N/A"


def segment_for(csv_path):
    """Return (segment, column prefix) from a segment keyword in the file name (case-insensitive)."""
    name = csv_path.stem.lower()
    matches = [seg for seg in SEGMENTS if seg.lower() in name]
    if len(matches) != 1:
        raise ValueError("File name must contain exactly one of 'Pelvis' or 'Trunk'")
    return matches[0], SEGMENTS[matches[0]]


def plot_csv(csv_path):
    segment, prefix = segment_for(csv_path)
    data = pd.read_csv(csv_path, encoding="utf-8-sig")
    # Accept accidental spaces in headers, e.g. "EE_Solver_ Fx".
    data.columns = data.columns.str.replace(" ", "", regex=False).str.strip()
    required = [f"{prefix}_{source}_{c}" for source in ("Solver", "Measured")
                for c in COMPONENTS]
    missing = [name for name in required if name not in data.columns]
    if missing:
        raise ValueError("Missing columns: " + ", ".join(missing))
    if data.empty:
        raise ValueError("CSV contains no samples")
    data[required] = data[required].apply(pd.to_numeric, errors="raise")

    if "Timestamp" in data.columns:
        timestamp = pd.to_numeric(data["Timestamp"], errors="raise")
        x = timestamp - timestamp.iloc[0]  # Logger timestamps are epoch seconds.
        xlabel = "Elapsed time (s)"
    else:
        x = range(len(data))
        xlabel = "Sample"

    if "ControlMode" not in data.columns:
        raise ValueError("Missing ControlMode column; metrics need assist-as-needed samples")
    regions = mode_regions(x, data["ControlMode"])
    # Metrics use only assist-as-needed samples; the plots still show the whole CSV.
    in_aan = (data["ControlMode"].fillna("").astype(str).str.strip().str.upper()
              == METRIC_MODE).to_numpy()
    aan_note = ("" if in_aan.any()
                else " (no assist-as-needed samples, metrics N/A)")

    fig, axes = plt.subplots(3, 2, figsize=(14, 11), sharex=True,
                             layout="constrained")
    fig.suptitle(f"{csv_path.name}\n{segment} ({prefix}) output wrench{aan_note}")
    if regions:
        present_modes = list(dict.fromkeys(mode for _, _, mode in regions))
        fig.legend(handles=[Patch(facecolor=MODE_STYLES.get(mode, (mode, "#e1bee7"))[1],
                                  alpha=0.2, label=MODE_STYLES.get(mode, (mode, ""))[0])
                            for mode in present_modes], loc="outside lower center",
                   ncol=2, fontsize=9)
    # Column order: Fx, Fy, Fz on the left; Tx, Ty, Tz on the right.
    for ax, component in zip(axes.T.flat, COMPONENTS):
        for start, end, mode in regions:
            ax.axvspan(start, end, color=MODE_STYLES.get(mode, (mode, "#e1bee7"))[1],
                       alpha=0.2, zorder=0)
        for start, _, _ in regions[1:]:
            ax.axvline(start, color="gray", linestyle="--", linewidth=0.7, alpha=0.6)
        solver = data[f"{prefix}_Solver_{component}"]
        measured = data[f"{prefix}_Measured_{component}"]
        ax.plot(x, solver, label="Commanded (solver)", linewidth=1.5)
        ax.plot(x, measured, label="Measured", linewidth=1.2)
        metrics = error_metrics(solver, measured, in_aan)
        unit = "N" if component.startswith("F") else "N m"
        m = {key: format_metric(metrics[key])
             for key in ("mae", "mae_pct", "max_abs", "max_error_pct")}
        if metrics["max_index"] is not None:
            index = metrics["max_index"]
            peak_x = x.iloc[index] if isinstance(x, pd.Series) else x[index]
            if np.isfinite(peak_x):
                # x is in data coordinates; y=0 is the bottom axis regardless of its limits.
                ax.plot([peak_x], [0], marker="|", markersize=16, markeredgewidth=2,
                        color="red", transform=ax.get_xaxis_transform(), clip_on=False)
                ax.annotate(f"index {index}", xy=(peak_x, 0),
                            xycoords=ax.get_xaxis_transform(), xytext=(0, 9),
                            textcoords="offset points", ha="center", va="bottom",
                            color="red", fontsize=8)
        ax.set_title(
            f"{component}\nAverage absolute error: {m['mae']} {unit} "
            f"({m['mae_pct']}% of mean |solver|) | "
            f"Max absolute error: {m['max_abs']} {unit}\n"
            f"Error at max-error sample: {m['max_error_pct']}% of |solver|",
            fontsize=9)
        ax.set_ylabel("Force (N)" if component.startswith("F") else "Torque (N m)")
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=8)
    for ax in axes[-1]:
        ax.set_xlabel(xlabel)
    return fig


def main():
    root = tk.Tk()
    root.withdraw()
    folder = filedialog.askdirectory(title="Select folder containing CSV logs")
    root.destroy()
    if not folder:
        return

    csv_files = sorted(p for p in Path(folder).iterdir()
                       if p.is_file() and p.suffix.lower() == ".csv")
    if not csv_files:
        messagebox.showinfo("No CSV files", "No CSV files found in the selected folder.")
        return

    for csv_path in csv_files:
        try:
            plot_csv(csv_path)
        except (ValueError, OSError, pd.errors.ParserError) as exc:
            messagebox.showwarning("CSV skipped", f"{csv_path.name}: {exc}")
            continue

    plt.show()


if __name__ == "__main__":
    main()
