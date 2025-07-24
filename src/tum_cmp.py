#!/usr/bin/env python3

import subprocess
import os
import shutil
from common import config

def tum_cmp():
    # --- Clean and create the output directory ---
    if os.path.exists(config.OUTPUT_TRAJ_FILES):
        shutil.rmtree(config.OUTPUT_TRAJ_FILES)
    os.makedirs(config.OUTPUT_TRAJ_FILES)

    # --- Tool 1: Plot Trajectories (evo_traj) ---
    print("--- [1/3] Running evo_traj to plot trajectories ---")
    subprocess.run([
        'evo_traj', 'tum',
        config.INPUT_REF_TRAJ,
        config.INPUT_EST_TRAJ,
        '--ref', config.INPUT_REF_TRAJ,
        '-v',
        '--plot',
        '--align_origin',  # Aligns start points for a cleaner plot
        '--sync',
        '--t_max_diff', '0.05'
    ], check=True)

    # --- Tool 2: Absolute Pose Error (evo_ape) ---
    print("\n--- [2/3] Running evo_ape for Absolute Pose Error ---")
    subprocess.run([
        'evo_ape', 'tum',
        config.INPUT_REF_TRAJ,
        config.INPUT_EST_TRAJ,
        '-va',
        '--plot',
        '--plot_mode', 'xyz',
        '--align',
        '--t_max_diff', '0.05',
        '--save_plot', os.path.join(config.OUTPUT_TRAJ_FILES, 'traj_plot.png')
    ], check=True)

    # --- Tool 3: Relative Pose Error (evo_rpe) ---
    print("\n--- [3/3] Running evo_rpe for Relative Pose Error ---")
    subprocess.run([
        'evo_rpe', 'tum',
        config.INPUT_REF_TRAJ,
        config.INPUT_EST_TRAJ,
        '-va',
        '--plot',
        '--align',
        '--t_max_diff', '0.05',
        '-d', '1',
        '--plot_x_dimension', 'seconds',
        '--save_plot', os.path.join(config.OUTPUT_TRAJ_FILES, 'rpe_plot.png'),
        '--save_results', os.path.join(config.OUTPUT_TRAJ_FILES, 'rpe_results.zip')
    ], check=True)

    print(f"\nEvaluation complete! Results saved in '{config.OUTPUT_TRAJ_FILES}'.")

if __name__ == "__main__":
    tum_cmp()