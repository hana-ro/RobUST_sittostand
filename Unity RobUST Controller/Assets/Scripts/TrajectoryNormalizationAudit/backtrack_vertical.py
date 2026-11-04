"""Compare exported group trajectories with raw NoVR trials, without modifying either.

Run with python -B backtrack_vertical.py PATH_TO_ZIP.
Errors use the exporter's 101-point sample-index time normalization.
"""
import re
import sys
import zipfile
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from trunk_trajectory_analysis import PENDULUM_LENGTHS

OUT = Path(__file__).resolve().parent
USER_LENGTH = 0.96
CHAIR_HEIGHT = 0.4318
TAU = np.linspace(0, 1, 101)


def main(archive):
    exported = {
        segment: pd.read_csv(ROOT / f"SitToWalk_GPR_{segment}_NoVR_normalized.csv")
        for segment in ("Pelvis", "Trunk")
    }
    errors, curves = [], []
    max_roundtrip_error = 0.0
    with zipfile.ZipFile(archive) as source:
        names = sorted(n for n in source.namelist()
                       if n.endswith("_trajectory_truncated.csv") and "_NoVR_" in n
                       and not Path(n).name.startswith("._") and "__MACOSX" not in n)
        for name in names:
            data = pd.read_csv(source.open(name))
            subject = int(re.search(r"P(\d+)", Path(name).parent.name).group(1))
            length = PENDULUM_LENGTHS[subject - 1]
            pelvis_start = data.Pelvis_Pos_Z.iloc[0]
            offset = pelvis_start - CHAIR_HEIGHT
            seated_height = CHAIR_HEIGHT + offset
            original_scale = length - seated_height
            for segment, column in (("Pelvis", "Pelvis_Pos_Z"), ("Trunk", "Chest_Pos_Z")):
                raw = data[column].to_numpy()
                raw_resampled = np.interp(TAU, np.linspace(0, 1, len(raw)), raw)
                # Inverting each trial's own normalization should be exact.
                normalized = (raw - raw[0]) / original_scale
                roundtrip = raw[0] + normalized * original_scale
                max_roundtrip_error = max(max_roundtrip_error, np.max(np.abs(raw - roundtrip)))
                csv = exported[segment]
                fraction = np.interp(TAU, csv.tau, csv.delta_Vertical_up_pct / 100)
                for scenario, scale_length in (("original_subject_length", length),
                                               ("user_length_0.96m", USER_LENGTH)):
                    scale = scale_length - seated_height
                    predicted = raw[0] + fraction * scale
                    error = predicted - raw_resampled
                    errors.append(dict(subject=subject, file=Path(name).name, segment=segment,
                        scenario=scenario, scale_length_m=scale_length, chair_height_m=CHAIR_HEIGHT,
                        pelvis_above_seat_m=offset, seated_pelvis_m=seated_height,
                        vertical_scale_m=scale, raw_start_m=raw[0], raw_end_m=raw[-1],
                        predicted_end_m=predicted[-1], raw_rise_cm=100*(raw[-1]-raw[0]),
                        predicted_rise_cm=100*(predicted[-1]-raw[0]),
                        rmse_cm=100*np.sqrt(np.mean(error**2)),
                        endpoint_error_cm=100*error[-1],
                        endpoint_abs_error_cm=100*abs(error[-1])))
                    for t, observed, reconstructed in zip(TAU, raw_resampled, predicted):
                        curves.append(dict(subject=subject, file=Path(name).name,
                            segment=segment, scenario=scenario, tau=t,
                            raw_height_m=observed, reconstructed_height_m=reconstructed,
                            raw_displacement_cm=100*(observed-raw[0]),
                            reconstructed_displacement_cm=100*(reconstructed-raw[0])))

    results = pd.DataFrame(errors)
    points = pd.DataFrame(curves)
    summary = results.groupby(["segment", "scenario"])[[
        "raw_rise_cm", "predicted_rise_cm", "rmse_cm", "endpoint_error_cm",
        "endpoint_abs_error_cm"]].mean()
    results.to_csv(OUT / "Backtrack_trial_errors.csv", index=False)
    points.to_csv(OUT / "Backtrack_vertical_curves.csv", index=False)
    summary.to_csv(OUT / "Backtrack_summary.csv")
    print("Maximum own-trial normalization roundtrip error (m):", max_roundtrip_error)
    print(summary.round(4).to_string())

    fig, axes = plt.subplots(1, 2, figsize=(12, 4.8))
    for ax, segment in zip(axes, ("Pelvis", "Trunk")):
        subset = points[points.segment == segment]
        native = subset[subset.scenario == "original_subject_length"]
        raw_mean = native.groupby("tau").raw_displacement_cm.mean()
        ax.plot(TAU*100, raw_mean, color="black", label="Raw recordings (mean)")
        for scenario, label, color in (
            ("original_subject_length", "CSV + original subject lengths", "tab:blue"),
            ("user_length_0.96m", "CSV + fixed 0.96 m length", "tab:orange")):
            mean = subset[subset.scenario == scenario].groupby("tau").reconstructed_displacement_cm.mean()
            ax.plot(TAU*100, mean, color=color, linestyle="--", label=label)
        ax.set(title=segment, xlabel="Movement (%)", ylabel="Vertical displacement from start (cm)")
        ax.axhline(0, color="0.7", linewidth=0.6)
        ax.grid(alpha=0.2)
        ax.legend(fontsize=8)
    fig.suptitle("45 NoVR trials: group-CSV reconstruction versus raw recordings")
    fig.tight_layout()
    fig.savefig(OUT / "Backtrack_mean_vertical.png", dpi=160)
    plt.close(fig)

    fig, axes = plt.subplots(3, 5, figsize=(16, 9), sharex=True, sharey=True)
    pelvis = points[(points.segment == "Pelvis") & (points.scenario == "original_subject_length")]
    for ax, subject in zip(axes.flat, sorted(pelvis.subject.unique())):
        subset = pelvis[pelvis.subject == subject]
        for _, trial in subset.groupby("file"):
            ax.plot(trial.tau*100, trial.raw_displacement_cm, color="0.65", linewidth=1)
        mean = subset.groupby("tau")[["raw_displacement_cm", "reconstructed_displacement_cm"]].mean()
        ax.plot(TAU*100, mean.raw_displacement_cm, color="black", label="Raw subject mean")
        ax.plot(TAU*100, mean.reconstructed_displacement_cm, color="tab:blue", linestyle="--", label="Group CSV reconstruction")
        ax.set_title(f"P{subject:02d}")
        ax.grid(alpha=0.2)
    axes[0, 0].legend(fontsize=7)
    fig.supxlabel("Movement (%)")
    fig.supylabel("Pelvis displacement from start (cm)")
    fig.suptitle("Original subject lengths: gray lines are raw trials; blue is the common group trajectory")
    fig.tight_layout()
    fig.savefig(OUT / "Backtrack_pelvis_by_subject.png", dpi=160)
    plt.close(fig)


if __name__ == "__main__":
    main(sys.argv[1])
