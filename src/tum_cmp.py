#!/usr/bin/env python3

import subprocess
import os
import shutil
import numpy as np
from evo.core import trajectory, sync, transformations
from evo.tools import file_interface
from common import config

def segment_wise_drift_analysis(ref_file, est_file, segment_len_m=200.0, t_max_diff=0.05):
    """
    Performs and prints a segment-wise Umeyama alignment to detect scale/rotation drift.
    """
    print(f"\n--- [4/4] Running Segment-Wise Drift Analysis (every {segment_len_m}m) ---")
    print("This checks for scale or rotation drift across the trajectory.")

    # 1. Load trajectories from files
    traj_ref = file_interface.read_tum_trajectory_file(ref_file)
    traj_est = file_interface.read_tum_trajectory_file(est_file)

    # 2. Synchronize trajectories based on timestamps
    traj_ref, traj_est = sync.associate_trajectories(traj_ref, traj_est, t_max_diff)

    # 3. Calculate cumulative distance along the reference trajectory
    positions = traj_ref.positions_xyz
    distances = np.cumsum(np.linalg.norm(np.diff(positions, axis=0), axis=1))
    distances = np.insert(distances, 0, 0)

    # 4. Determine the indices where each new segment should start
    split_indices = [0]
    next_split_dist = segment_len_m
    for i, dist in enumerate(distances):
        if dist >= next_split_dist:
            split_indices.append(i)
            next_split_dist += segment_len_m
    if split_indices[-1] != len(traj_ref.timestamps) - 1:
        split_indices.append(len(traj_ref.timestamps) - 1)

    # 5. Perform alignment on each segment and print the results
    print("\n" + "="*55)
    print(f"{'Segment #':<10} | {'Distance Range (m)':<20} | {'Scale':<10} | {'Yaw (deg)':<10}")
    print("-"*55)

    for i in range(len(split_indices) - 1):
        start_idx = split_indices[i]
        end_idx = split_indices[i+1]

        if end_idx - start_idx < 3: # Need at least 3 points for superimposition
            continue

        ref_segment_xyz = traj_ref.positions_xyz[start_idx:end_idx]
        est_segment_xyz = traj_est.positions_xyz[start_idx:end_idx]
        
        # 1. Get the 4x4 transformation matrix.
        transform_matrix = transformations.superimposition_matrix(
            est_segment_xyz.T, ref_segment_xyz.T, scale=True
        )

        # 2. Decompose the matrix to get scale and rotation angles.
        scale_factors, shear, angles, translate, perspective = transformations.decompose_matrix(
            transform_matrix
        )

        # The scale is the first factor (should be uniform for a similarity transform).
        scale = scale_factors[0]
        # The yaw angle is the third Euler angle (rotation about z).
        yaw_rad = angles[2]
        yaw_deg = np.degrees(yaw_rad)

        dist_range_str = f"{distances[start_idx]:.1f} - {distances[end_idx]:.1f}"
        print(f"{i+1:<10} | {dist_range_str:<20} | {scale:<10.4f} | {yaw_deg:<10.2f}")

    print("="*55)
    print("--> A consistent 'Scale' and 'Yaw' indicates low drift.")
    print("--> A drifting 'Scale' or 'Yaw' points to uncorrected SLAM errors.")

def tum_cmp():
    # --- Clean and create the output directory ---
    if os.path.exists(config.OUTPUT_TRAJ_FILES):
        shutil.rmtree(config.OUTPUT_TRAJ_FILES)
    os.makedirs(config.OUTPUT_TRAJ_FILES)

    # --- Tool 1: Plot Trajectories (evo_traj) ---
    print("--- [1/4] Running evo_traj to plot trajectories ---")
    subprocess.run([
        'evo_traj', 'tum',
        config.INPUT_REF_TRAJ,
        config.INPUT_EST_TRAJ,
        '--ref', config.INPUT_REF_TRAJ,
        '-v',
        '--plot',
        '--plot_mode', 'xy',
        '--align_origin',  # Aligns start points for a cleaner plot
        '--sync',
        '--t_max_diff', '0.05'
    ], check=True)

    # --- Tool 2: Absolute Pose Error (evo_ape) ---
    print("\n--- [2/4] Running evo_ape for Absolute Pose Error ---")
    subprocess.run([
        'evo_ape', 'tum',
        config.INPUT_REF_TRAJ,
        config.INPUT_EST_TRAJ,
        '-va',
        '--plot',
        '--plot_mode', 'xy',
        '--align',
        '--t_max_diff', '0.05',
        '--save_plot', os.path.join(config.OUTPUT_TRAJ_FILES, 'traj_plot.png')
    ], check=True)

    # --- Tool 3: Relative Pose Error (evo_rpe) ---
    print("\n--- [3/4] Running evo_rpe for Relative Pose Error ---")
    subprocess.run([
        'evo_rpe', 'tum',
        config.INPUT_REF_TRAJ,
        config.INPUT_EST_TRAJ,
        '-va',
        '--plot',
        '--plot_mode', 'xy',
        '--align',
        '--t_max_diff', '0.05',
        '-d', '1',
        '--plot_x_dimension', 'seconds',
        '--save_plot', os.path.join(config.OUTPUT_TRAJ_FILES, 'rpe_plot.png'),
        '--save_results', os.path.join(config.OUTPUT_TRAJ_FILES, 'rpe_results.zip')
    ], check=True)
    
    # --- Tool 4: Segment-wise Drift Analysis (NEW) ---
    segment_wise_drift_analysis(
        config.INPUT_REF_TRAJ,
        config.INPUT_EST_TRAJ,
        segment_len_m=200.0,
        t_max_diff=0.05
    )

    print(f"\nEvaluation complete! Results saved in '{config.OUTPUT_TRAJ_FILES}'.")

if __name__ == "__main__":
    tum_cmp()