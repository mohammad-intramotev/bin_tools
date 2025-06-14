#!/usr/bin/env python3

import os
import subprocess


if __name__ == "__main__":
    command = input("""
        Welcome to bin_tools. Please select a tool:\n
        1) Combine multiple .bin files
        2) Convert .bin to ROS bag
        3) Create ground truth using GPS from .bin and convert to TUM format
        4) Compare trajectories
        5) Diagnose .bin file\n
        """)
    
    tools = {"1": "bin_merge", "2": "bin_to_ros", "3": "bin_to_ros", "4": "bin_to_tum", "5": "bin_diag"}

    env = {**os.environ, "COMMAND": tools[command]}

    try:
        subprocess.run(["docker", "compose", "up", "--build"], env=env, check=True)
    except subprocess.CalledProcessError:
        print(f"Failed to build distro.")