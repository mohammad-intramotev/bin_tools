#!/usr/bin/env python3

import os
import subprocess


if __name__ == "__main__":
    command = input("""
        Welcome to bin_tools. Please select a tool:\n
        1) Combine multiple .bin files
        2) Convert .bin to ROS 1 bag with custom messages
        3) Convert .bin to ROS 2 bag with custom messages
        4) Convert .bin to ROS 2 bag with standard messages
        5) Create ground truth using GPS data from .bin and convert to TUM format
        6) Compare trajectories
        7) Diagnose .bin file\n
        """)
    
    env = {
        **os.environ,
        "ROS_DISTRO": "humble" if command in ("3", "4") else "noetic",
        "ROS_VER": "ros2" if command in ("3", "4") else "ros1",
        "COMMAND": command,
    }
    try:
        subprocess.run(["docker", "compose", "up", "--build",  "--remove-orphans"], env=env, check=True)
    except subprocess.CalledProcessError:
        print(f"Failed to build distro.")