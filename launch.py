#!/usr/bin/env python3

import os
import subprocess


def bin_merge_command():
    env = {**os.environ, "COMMAND": "bin_merge"}

    try:
        subprocess.run(["docker", "compose", "up", "--build"], env=env, check=True)
    except subprocess.CalledProcessError:
        print(f"Failed to build distro.")

def bin_to_rosbag_command():
    distros = {"1":"noetic", "2":"humble"}
    selection = input("\nPlease enter output format (1 for ROS 1 bag, 2 for ROS 2 bag): ")

    while selection not in distros:
        selection = input("Invalid input. Please enter output format (1 for ROS 1, 2 for ROS 2): ")

    env = {**os.environ, "ROS_DISTRO": distros[selection], "COMMAND": "bin_to_ros"}
 
    try:
        subprocess.run(["docker", "compose", "up", "--build"], env=env, check=True)
    except subprocess.CalledProcessError:
        print(f"Failed to build distro: {distros[selection]}")

def bin_to_tum_command():
    env = {**os.environ, "COMMAND": "bin_to_tum"}

    try:
        subprocess.run(["docker", "compose", "up", "--build"], env=env, check=True)
    except subprocess.CalledProcessError:
        print(f"Failed to build distro")

def traj_cmp_command():
    env = {**os.environ, "COMMAND": "traj_cmp"}

    try:
        subprocess.run(["docker", "compose", "up", "--build"], env=env, check=True)
    except subprocess.CalledProcessError:
        print(f"Failed to build distro")


if __name__ == "__main__":
    command = input("""
        Welcome to bin_tools. Please select a tool:\n
        1) Combine multiple .bin files
        2) Convert .bin to ROS bag
        3) Create ground truth using GPS from .bin and convert to TUM format
        4) Compare trajectories\n
        """)

    match command:
        case "1":
            bin_merge_command()
        case "2":
            bin_to_rosbag_command()
        case "3":
            bin_to_tum_command()
        case "4":
            traj_cmp_command()
        case _:
            print("Invalid choice. Exiting!")