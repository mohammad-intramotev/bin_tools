#!/usr/bin/env python3

import subprocess
import os
from common import config

os.makedirs(config.OUTPUT_TRAJ_FILES, exist_ok=True)


# --- Tool 1: Plot Trajectories (evo_traj) ---
print("--- [1/3] Running evo_traj to plot trajectories ---")
subprocess.run([
    'evo_traj',
    'tum',
    config.INPUT_TRAJ_1,
    config.INPUT_TRAJ_2,
    '--ref', config.INPUT_TRAJ_1,
    '-va',
    '--plot',
], check=True)


# --- Tool 2: Absolute Pose Error (evo_ape) ---
print("\n--- [2/3] Running evo_ape for Absolute Pose Error ---")
subprocess.run([
    'evo_ape',
    'tum',
    config.INPUT_TRAJ_1,
    config.INPUT_TRAJ_2,
    '-va',
    '--plot',
    '--plot_mode', 'xyz',
    '--save_plot', os.path.join(config.OUTPUT_TRAJ_FILES, 'traj_plot.png')
], check=True)


# --- Tool 3: Relative Pose Error (evo_rpe) ---
print("\n--- [3/3] Running evo_rpe for Relative Pose Error ---")
subprocess.run([
    'evo_rpe',
    'tum',
    config.INPUT_TRAJ_1,
    config.INPUT_TRAJ_2,
    '-va',
    '--plot',
    '--save_plot', os.path.join(config.OUTPUT_TRAJ_FILES, 'rpe_plot.png'),
    '--save_results', os.path.join(config.OUTPUT_TRAJ_FILES, 'rpe_results.zip')
], check=True)

print(f"\nEvaluation complete! Results saved in '{config.OUTPUT_TRAJ_FILES}'.")