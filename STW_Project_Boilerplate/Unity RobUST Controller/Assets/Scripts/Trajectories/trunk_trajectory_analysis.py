"""
Dual-Segment (Pelvis & Trunk) Trajectory Analysis with GPR & Anthropometric Normalization.

Based on trunk_trajectory_analysis.m. AP and ML are normalized by leg length L (pelvis tracker to
floor, standing). Vertical is normalized by rise height H = L - seated pelvis height, so that Unity
can account for each subject's leg length and chair. Exports the GPR mean trajectories and pooled
SD (within-subject GPR + between-subject) for Unity / RobUST simulation.

Run from the folder the CSVs should land in (Assets/Scripts, where RobotController reads them):
    python trunk_trajectory_analysis.py
"""

import os
import re
import sys

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib.lines import Line2D
from scipy import stats as scipy_stats
from scipy.linalg import cho_factor, cho_solve
from scipy.optimize import minimize

N_TIME_POINTS = 101  # 0-100%
ALPHA = 0.05

# Anthropometric Normalization Mapping: Effective Pendulum Length (m)
PENDULUM_LENGTHS = [1.11, 0.97, 1.08, 1.07, 1.04, 1.09, 1.04, 1.12, 1.21, 1.21, 1.12, 1.08, 0.98, 0.98, 1.02]

# Universal color mapping
CONDITION_COLORS = {
    'NoVR':          np.array([254, 192, 170]) / 255,
    'VR':            np.array([248, 150, 119]) / 255,
    'Neutral':       np.array([241, 107,  67]) / 255,
    'Flexion':       np.array([236,  78,  32]) / 255,
    'Extension':     np.array([184,  97,  38]) / 255,
    'Rotation':      np.array([132, 115,  43]) / 255,
    'RightRotation': np.array([132, 115,  43]) / 255,
    'Allocentric':   np.array([139, 152, 103]) / 255,
    'Stroop':        np.array([146, 189, 163]) / 255,
    'Math Equation': np.array([ 87, 124,  91]) / 255,
    'Equation':      np.array([ 87, 124,  91]) / 255,
    'TMT':           np.array([ 28,  58,  19]) / 255,
    'TrailMakingA':  np.array([ 28,  58,  19]) / 255,
}

CONDITION_GROUPS = [
    ['NoVR'],
    ['VR', 'Neutral', 'Flexion', 'Extension', 'RightRotation', 'Allocentric'],
    ['VR', 'Equation', 'Stroop', 'TrailMakingA'],
]

GROUP_NAMES = ['NoVR', 'Head-Oriented', 'Head-Oriented-Cognitive']

SEGMENTS = ('pelvis', 'trunk')
TRUNK_NAMES = ('chest', 'sternum', 'thorax', 'trunk')


def main(display_option=None, main_folder=None):
    print('=== Dual-Segment (Pelvis & Trunk) Trajectory Analysis (Pendulum Length Normalized) ===')

    # Step 0: User input for display option
    print('\nDisplay Options:')
    print('  1. Full trajectories (group mean + confidence bands)')
    print('  2. Phase-separated (both Preparation & Transition phases)')
    print('  3. Transition phase only (Phase 2: Seat-Off to Gait Initiation) [Exports for RobUST]')
    while display_option not in (1, 2, 3):
        try:
            display_option = int(input('Select display option (1, 2, or 3): '))
        except ValueError:
            display_option = 0

    if display_option == 1:
        print('Selected: Full trajectories group analysis')
        separate_by_phase = False
        transition_only = False
    elif display_option == 2:
        print('Selected: Phase-separated group analysis (both phases)')
        separate_by_phase = True
        transition_only = False
    else:
        print('Selected: Transition phase only group analysis')
        separate_by_phase = True
        transition_only = True

    print(f'Time normalization: All trajectories resampled to {N_TIME_POINTS} points')

    # Step 1: Select main folder containing subject subfolders
    print('\nStep 1: Select main folder containing subject subfolders')
    if main_folder is None:
        main_folder = select_folder()
    if not main_folder:
        print('Folder selection cancelled')
        return

    subject_pendulum_lengths = {}
    for idx, val in enumerate(PENDULUM_LENGTHS, start=1):
        # Map 3-digit (P001), 2-digit (P01), and 1-digit (P1) styles
        subject_pendulum_lengths[f'P{idx:03d}'] = val
        subject_pendulum_lengths[f'P{idx:02d}'] = val
        subject_pendulum_lengths[f'P{idx}'] = val
    default_length = float(np.mean(PENDULUM_LENGTHS))  # 1.075m nominal fallback / group reference

    # Get all subject folders (ignoring hidden folders starting with '.' or '._')
    subject_folders = [name for name in sorted(os.listdir(main_folder))
                       if os.path.isdir(os.path.join(main_folder, name)) and not name.startswith('.')]

    if not subject_folders:
        raise RuntimeError('No subject folders found in the selected directory')

    print(f'Found {len(subject_folders)} subject folders')

    # Step 2: Parse all subjects' data
    print("\nStep 2: Loading data from all subjects...")

    all_subjects_data = {}
    subject_names = []

    for subject_folder in subject_folders:
        subject_path = os.path.join(main_folder, subject_folder)

        # Extract subject ID (e.g., P01, P13, P001)
        subject_match = re.search(r'P\d+', subject_folder)
        if subject_match is None:
            print(f'Warning: Skipping folder (no subject ID found): {subject_folder}')
            continue

        subject_id = subject_match.group()
        subject_names.append(subject_id)
        print(f'  Processing subject: {subject_id}')

        # Get all truncated CSV files in this subject's folder
        csv_files = [name for name in sorted(os.listdir(subject_path))
                     if name.endswith('_trajectory_truncated.csv') and not name.startswith('._')]

        if not csv_files:
            print(f'    Warning: No valid truncated CSV files found for {subject_id}')
            continue

        # Parse files and organize by condition
        for filename in csv_files:
            base_name = filename.split('_trajectory_truncated.csv')[0]
            parts = base_name.split('_')

            if len(parts) >= 2:
                condition = parts[1]
                all_subjects_data.setdefault(condition, {}).setdefault(subject_id, []).append(
                    os.path.join(subject_path, filename))

    subject_names = sorted(set(subject_names))
    n_subjects = len(subject_names)
    print(f'\nSuccessfully loaded data from {n_subjects} subjects')

    # Get all conditions
    conditions = list(all_subjects_data)
    print(f"Conditions found: {', '.join(conditions)}")

    # Step 4: Calculate subject-level GPR with Anthropometric Normalization
    print('\nStep 3: Computing subject-level GPR for Pelvis & Trunk (Normalized by Pendulum Length)...')

    subject_gpr_results = {}
    query_time = np.linspace(0, 1, N_TIME_POINTS)

    for cond_name in conditions:
        print(f'  Condition: {cond_name}')
        cond_results = subject_gpr_results.setdefault(cond_name, {})

        if not separate_by_phase:
            # Full trajectory GPR for each subject
            for subject_id in subject_names:
                if subject_id not in all_subjects_data[cond_name]:
                    continue

                # Resolve subject pendulum length
                if subject_id in subject_pendulum_lengths:
                    L = subject_pendulum_lengths[subject_id]
                else:
                    L = default_length
                    print(f'    Warning: Pendulum length missing for {subject_id}. Using default {L:.2f}m')

                fit_subject(all_subjects_data[cond_name][subject_id], L, None, query_time, subject_id, cond_results)
        else:
            # Phase-separated GPR
            for phase in (1, 2):
                phase_results = cond_results.setdefault(f'phase{phase}', {})
                for subject_id in subject_names:
                    if subject_id not in all_subjects_data[cond_name]:
                        continue

                    L = subject_pendulum_lengths.get(subject_id, default_length)
                    fit_subject(all_subjects_data[cond_name][subject_id], L, phase, query_time, subject_id, phase_results)
                print(f'    Phase {phase}: Dual-segment GPR complete for all subjects')

    # Step 5: Calculate group-level statistics
    print('\nStep 4: Computing group-level statistics for Pelvis & Trunk...')
    group_results = {}

    for cond_name in conditions:
        cond_gpr = subject_gpr_results[cond_name]
        if not cond_gpr:
            continue

        cond_group = group_results.setdefault(cond_name, {})
        if not separate_by_phase:
            for segment in SEGMENTS:
                if segment in cond_gpr:
                    cond_group[segment] = compute_segment_group_stats(cond_gpr[segment], subject_names)
            print(f'  {cond_name}: Group stats computed for Pelvis and Trunk')
        else:
            for phase in (1, 2):
                phase_field = f'phase{phase}'
                phase_gpr = cond_gpr[phase_field]
                if not phase_gpr:
                    continue

                phase_group = cond_group.setdefault(phase_field, {})
                for segment in SEGMENTS:
                    if segment in phase_gpr:
                        phase_group[segment] = compute_segment_group_stats(phase_gpr[segment], subject_names)
                print(f'  {cond_name} Phase {phase}: Group stats computed for Pelvis and Trunk')

    # Step 5.5: Export GPR Mean Trajectories to CSV for Unity / RobUST
    print('\nExporting Pelvis and Trunk GPR mean trajectories to CSV for Unity/RobUST...')

    target_condition = 'NoVR'
    if target_condition not in group_results and conditions:
        target_condition = conditions[0]

    tau = np.linspace(0, 1, N_TIME_POINTS)

    if target_condition in group_results:
        if separate_by_phase:
            # Phase selection based on user preference
            pelvis_phase = 2  # Transition phase default for pelvis
            trunk_phase = 1   # Preparation phase default for trunk (or 2 if transition_only)
            if transition_only:
                trunk_phase = 2

            pelvis_src = group_results[target_condition][f'phase{pelvis_phase}']['pelvis']
            trunk_src = group_results[target_condition][f'phase{trunk_phase}']['trunk']
            pelvis_label = f'Pelvis (Phase {pelvis_phase})'
            trunk_label = f'Trunk (Phase {trunk_phase})'
        else:
            pelvis_src = group_results[target_condition]['pelvis']
            trunk_src = group_results[target_condition]['trunk']
            pelvis_label = 'Full Pelvis'
            trunk_label = 'Full Trunk'

        export_segment_csv(tau, pelvis_src, f'SitToWalk_GPR_Pelvis_{target_condition}_normalized.csv')
        print(f'  --> Saved {pelvis_label} trajectory to SitToWalk_GPR_Pelvis_{target_condition}_normalized.csv')

        export_segment_csv(tau, trunk_src, f'SitToWalk_GPR_Trunk_{target_condition}_normalized.csv')
        print(f'  --> Saved {trunk_label} trajectory to SitToWalk_GPR_Trunk_{target_condition}_normalized.csv')

    # Step 6: Plot group results - SAGITTAL PLANE (Pelvis & Trunk Simultaneous)
    print('\nStep 5: Creating publication-ready sagittal plane plots (Pelvis vs. Trunk)...')

    for group_conditions, group_name in zip(CONDITION_GROUPS, GROUP_NAMES):
        print(f'  Creating {group_name}')

        if not separate_by_phase:
            # Full trajectory mode - single plot (Sagittal only)
            sources = [(cond, group_results[cond]) for cond in group_conditions if cond in group_results]
            handles = plot_sagittal(sources, f'{group_name} - Full Trajectory (Pelvis & Trunk)', 20)
            if handles:
                create_separate_legend(handles)
        else:
            # Phase-separated mode - plots (Sagittal only)
            phases_to_plot = (2,) if transition_only else (1, 2)

            for p_idx, phase in enumerate(phases_to_plot):
                phase_field = f'phase{phase}'
                sources = [(cond, group_results[cond][phase_field]) for cond in group_conditions
                           if cond in group_results and phase_field in group_results[cond]]
                phase_name = 'Preparation Phase (Pre-Seat-Off)' if phase == 1 else 'Transition Phase (Post-Seat-Off)'
                handles = plot_sagittal(sources, f'{group_name} - {phase_name}', 22)
                if p_idx == 0 and handles:
                    create_separate_legend(handles)

    print('\n=== Analysis Complete ===')
    plt.show()


# =========================================================================
# Helper Functions
# =========================================================================

def select_folder():
    import tkinter as tk
    from tkinter import filedialog

    root = tk.Tk()
    root.withdraw()
    folder = filedialog.askdirectory(title='Select main folder containing subject folders (e.g., P01_truncated, P13_truncated, ...)')
    root.destroy()
    return folder


def find_columns(columns, *names):
    """Mirrors find(contains(vars, name, 'IgnoreCase', true) & contains(vars, axis, 'IgnoreCase', true), 1)."""
    found = []
    for axis in ('x', 'y', 'z'):
        match = next((col for col in columns
                      if any(name in col.lower() for name in names) and axis in col.lower()), None)
        found.append(match)
    return None if None in found else found


def normalize_trial(data, columns, phase_mask, L, rise_height, target_time):
    """Zeroes a trial at its first sample, scales X/Y by L and Z by rise height, and resamples to 0-100%."""
    resampled = []
    for col, scale in zip(columns, (L, L, rise_height)):
        raw = data[col].to_numpy(dtype=float)
        if phase_mask is not None:
            raw = raw[phase_mask]
        if raw.size == 0:
            return None
        norm = ((raw - raw[0]) / scale) * 100
        resampled.append(np.interp(target_time, np.linspace(0, 1, raw.size), norm))
    return resampled


def fit_subject(file_list, L, phase, query_time, subject_id, results):
    """Fits the pelvis and trunk GPRs for one subject, across all of their trials in a condition."""
    target_time = np.linspace(0, 1, N_TIME_POINTS)
    trials = {segment: [] for segment in SEGMENTS}

    for filepath in file_list:
        data = pd.read_csv(filepath, encoding='utf-8-sig')
        columns = list(data.columns)

        phase_mask = None
        if phase is not None:
            phase_col = next((col for col in columns if 'phase' in col.lower()), None)
            if phase_col is None:
                raise RuntimeError(f'Phase column missing in: {filepath}')
            phase_mask = data[phase_col].to_numpy() == phase

        segment_columns = {'pelvis': find_columns(columns, 'pelvis'), 'trunk': find_columns(columns, *TRUNK_NAMES)}
        if segment_columns['pelvis'] is None:
            raise RuntimeError(f'Pelvis columns missing in: {filepath}')

        # Both segments share the pelvis rise height. Pelvis Z is floor-referenced and the trial starts
        # seated, so the first sample is the seated height, even when fitting a later phase.
        rise_height = L - data[segment_columns['pelvis'][2]].iloc[0]
        if not rise_height > 0:
            print(f'    Warning: Skipping trial that does not start seated (leg length {L:.2f}m): {os.path.basename(filepath)}')
            continue

        for segment, cols in segment_columns.items():
            if cols is not None:
                trial = normalize_trial(data, cols, phase_mask, L, rise_height, target_time)
                if trial is not None:
                    trials[segment].append(trial)

    for segment, segment_trials in trials.items():
        if not segment_trials:
            continue

        time_all = np.tile(target_time, len(segment_trials))
        stacked = np.asarray(segment_trials)  # trials x axes x time
        fit = {}
        for axis_idx, axis in enumerate(('x', 'y', 'z')):
            pred, sd = fit_and_predict_gpr(time_all, stacked[:, axis_idx, :].ravel(), query_time)
            fit[f'{axis}_pred'] = pred
            fit[f'{axis}_sd'] = sd
        results.setdefault(segment, {})[subject_id] = fit


def sq_exp_kernel(a, b, length, signal):
    return signal ** 2 * np.exp(-0.5 * ((a[:, None] - b[None, :]) / length) ** 2)


def group_replicates(time_all, val_all):
    """Collapses repeated time points. Trials share a time grid, so this keeps the solves at 101x101."""
    unique_time, inverse, counts = np.unique(time_all, return_inverse=True, return_counts=True)
    means = np.bincount(inverse, weights=val_all) / counts
    within_ss = float(np.sum((val_all - means[inverse]) ** 2))
    return unique_time, means, counts, within_ss


def gpr_solve(unique_time, means, counts, length, signal, sigma):
    """Factors K + sigma^2/n and solves for the constant basis, which fitrgp estimates by GLS."""
    K = sq_exp_kernel(unique_time, unique_time, length, signal)
    K[np.diag_indices_from(K)] += sigma ** 2 / counts
    factor = cho_factor(K, lower=True)
    ones = np.ones_like(means)
    A_inv_ones = cho_solve(factor, ones)
    A_inv_means = cho_solve(factor, means)
    beta = (ones @ A_inv_means) / (ones @ A_inv_ones)
    residual = means - beta
    return factor, beta, residual, cho_solve(factor, residual)


def negative_log_likelihood(log_params, unique_time, means, counts, within_ss, n_points):
    length, signal, sigma = np.exp(log_params)
    try:
        factor, _, residual, alpha = gpr_solve(unique_time, means, counts, length, signal, sigma)
    except np.linalg.LinAlgError:
        return np.inf

    log_det = (2.0 * np.sum(np.log(np.diag(factor[0])))
               + np.sum(np.log(counts))
               + 2.0 * (n_points - counts.size) * np.log(sigma))
    quadratic = residual @ alpha + within_ss / sigma ** 2
    return 0.5 * (quadratic + log_det + n_points * np.log(2.0 * np.pi))


def fit_and_predict_gpr(time_all, val_all, query_time, return_params=False):
    """
    Equivalent of fitrgp(..., 'FitMethod', 'sd', 'PredictMethod', 'exact', 'Sigma', 0.1,
    'SigmaLowerBound', 0.01) followed by predict(): squared exponential kernel, constant basis,
    fitrgp's default initial kernel parameters, and an sd that includes observation noise.
    """
    valid = ~(np.isnan(time_all) | np.isnan(val_all))
    time_all = time_all[valid]
    val_all = val_all[valid]

    initial = np.log([np.std(time_all, ddof=1), max(np.std(val_all, ddof=1) / np.sqrt(2.0), 1e-6), 0.1])

    # 'sd' fits the hyperparameters on an active set of min(1000, n) points
    if time_all.size > 1000:
        active = np.random.default_rng(0).choice(time_all.size, 1000, replace=False)
    else:
        active = np.arange(time_all.size)
    active_groups = group_replicates(time_all[active], val_all[active])

    result = minimize(negative_log_likelihood, initial, args=(*active_groups, active.size), method='L-BFGS-B',
                      bounds=[(None, None), (None, None), (np.log(0.01), None)])
    length, signal, sigma = np.exp(result.x)

    # 'exact' prediction conditions on all of the data
    unique_time, means, counts, _ = group_replicates(time_all, val_all)
    factor, beta, _, alpha = gpr_solve(unique_time, means, counts, length, signal, sigma)
    K_star = sq_exp_kernel(query_time, unique_time, length, signal)
    pred = beta + K_star @ alpha
    variance = signal ** 2 + sigma ** 2 - np.sum(K_star * cho_solve(factor, K_star.T).T, axis=1)
    sd = np.sqrt(np.maximum(variance, 0.0))

    if return_params:
        return pred, sd, (length, signal, sigma, beta)
    return pred, sd


def compute_segment_group_stats(subject_dict, subject_names):
    valid = [subject_dict[s_id] for s_id in subject_names if s_id in subject_dict and subject_dict[s_id]['x_pred'].size]
    n_val = len(valid)
    if n_val == 0:
        return {}

    t_crit = scipy_stats.t.ppf(1 - ALPHA / 2, max(1, n_val - 1))
    stats = {}
    for axis in ('x', 'y', 'z'):
        pred_mat = np.column_stack([fit[f'{axis}_pred'] for fit in valid])
        sd_mat = np.column_stack([fit[f'{axis}_sd'] for fit in valid])

        mean = np.nanmean(pred_mat, axis=1)
        sd = np.sqrt(np.mean(sd_mat ** 2 + (pred_mat - mean[:, None]) ** 2, axis=1))

        stats[f'{axis}_pred'] = mean
        stats[f'{axis}_sd'] = sd
        stats[f'{axis}_ci_lower'] = mean - t_crit * sd / np.sqrt(n_val)
        stats[f'{axis}_ci_upper'] = mean + t_crit * sd / np.sqrt(n_val)
    stats['n_subjects'] = n_val
    return stats


def export_segment_csv(tau, src, filename):
    """AP and ML are % leg length, vertical is % rise height. RobotController converts these to meters."""
    pd.DataFrame({
        'tau': tau,
        'delta_AP_forward_pct': src['y_pred'],
        'delta_Vertical_up_pct': src['z_pred'],
        'delta_ML_left_pct': src['x_pred'],
        'sd_AP_pct': src['y_sd'],
        'sd_Vertical_pct': src['z_sd'],
        'sd_ML_pct': src['x_sd'],
    }).to_csv(filename, index=False, float_format='%.15g')


def plot_sagittal(sources, title, axis_label_size):
    """Plots pelvis (solid) and trunk (dashed) for each (condition, results) pair; returns legend entries."""
    fig, ax = plt.subplots(figsize=(8.5, 6.0), facecolor='w')
    legend_handles = []

    for cond_name, results in sources:
        color = CONDITION_COLORS.get(cond_name, np.array([0.5, 0.5, 0.5]))

        # Plot Pelvis (Solid Line)
        if 'pelvis' in results:
            p_res = results['pelvis']
            ax.fill(np.r_[p_res['y_ci_upper'], p_res['y_ci_lower'][::-1]],
                    np.r_[p_res['z_ci_upper'], p_res['z_ci_lower'][::-1]],
                    color=color, alpha=0.22, edgecolor='none')
            line, = ax.plot(p_res['y_pred'], p_res['z_pred'], '-', color=color, linewidth=3,
                            label=f'{cond_name} (Pelvis)')
            legend_handles.append(line)

        # Plot Trunk (Dashed Line)
        if 'trunk' in results:
            t_res = results['trunk']
            trunk_tint = color * 0.75  # Slightly deeper tint for trunk contrast
            ax.fill(np.r_[t_res['y_ci_upper'], t_res['y_ci_lower'][::-1]],
                    np.r_[t_res['z_ci_upper'], t_res['z_ci_lower'][::-1]],
                    color=trunk_tint, alpha=0.15, edgecolor='none')
            line, = ax.plot(t_res['y_pred'], t_res['z_pred'], '--', color=trunk_tint, linewidth=2.5,
                            label=f'{cond_name} (Trunk)')
            legend_handles.append(line)

    ax.grid(True)
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    ax.tick_params(direction='out', length=0, labelsize=18)
    for spine in ax.spines.values():
        spine.set_linewidth(1)

    ax.set_xlabel('AP Displacement (% Pendulum Length)', fontsize=axis_label_size, fontweight='bold')
    ax.set_ylabel('Vertical Displacement (% Rise Height)', fontsize=axis_label_size, fontweight='bold')
    ax.set_title(title, fontsize=18, fontweight='bold')
    ax.autoscale(tight=True)
    return legend_handles


def create_separate_legend(legend_handles):
    fig, ax = plt.subplots(figsize=(3.6, 2.6), facecolor='w')
    ax.axis('off')
    proxies = [Line2D([], [], linestyle=h.get_linestyle(), color=h.get_color(), linewidth=h.get_linewidth())
               for h in legend_handles]
    ax.legend(proxies, [h.get_label() for h in legend_handles], loc='best', fontsize=14, frameon=False)


if __name__ == '__main__':
    sys.exit(main())
