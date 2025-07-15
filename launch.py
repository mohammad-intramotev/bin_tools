#!/usr/bin/env python3

import os
import subprocess


if __name__ == "__main__":
    command = input("""
        Welcome to bin_tools. Please select a tool:\n
        1) Combine multiple .bin files
        2) Convert .bin to ROS1 bag
        3) Convert .bin to ROS2 bag
        4) Create ground truth using GPS from .bin and convert to TUM format
        5) Compare trajectories
        6) Diagnose .bin file\n
        """)
    
    tools = {"1": "bin_merge", "2": "bin_to_ros", "3": "bin_to_ros", "4": "bin_to_tum", "5": "traj_compare", "6": "bin_diag"}

    env = {
        **os.environ,
        "ROS_DISTRO": "humble" if command == "3" else "noetic",
        "ROS_VER": "ros2" if command == "3" else "ros1",
        "COMMAND": tools[command],
    }
    try:
        subprocess.run(["docker", "compose", "up", "--build",  "--remove-orphans"], env=env, check=True)
    except subprocess.CalledProcessError:
        print(f"Failed to build distro.")