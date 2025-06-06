#!/usr/bin/env python3

import os
import subprocess


def bin_merge_command():
    env = {**os.environ, "COMMAND": "bin_merge"}

    try:
        subprocess.run(["docker", "compose", "up", "--build"], env=env, check=True)
    except subprocess.CalledProcessError:
        print(f"Failed to build distro.")

def output_format():
    selection = input("Please enter output format (1 for ROS 1 bag, 2 for ROS 2 bag): ")

    while selection not in ('1', '2'):
        selection = input("Invalid input. Please enter output format (1 for ROS 1, 2 for ROS 2): ")

    return "noetic" if selection == '1' else "humble"

def build_ros_image(selection):
    env = {**os.environ, "ROS_DISTRO": selection, "COMMAND": "bin_convert"}
 
    try:
        subprocess.run(["docker", "compose", "up", "--build"], env=env, check=True)
    except subprocess.CalledProcessError:
        print(f"Failed to build distro: {selection}")

def bin_to_tum_command():
    env = {**os.environ, "COMMAND": "bin_to_tum_command"}

    try:
        subprocess.run(["docker", "compose", "up", "--build"], env=env, check=True)
    except subprocess.CalledProcessError:
        print(f"Failed to build distro")

def traj_cmp_command():
    env = {**os.environ, "COMMAND": "traj_cmp_command"}

    try:
        subprocess.run(["docker", "compose", "up", "--build"], env=env, check=True)
    except subprocess.CalledProcessError:
        print(f"Failed to build distro")


if __name__ == "__main__":
    command = input("""
        Welcome to bin_tools. Please select a tool:\n
        1) Combine multiple .bin files
        2) Convert .bin to ROS bag
        3) Compare trajectories
        4) Create ground truth using GPS from .bin and convert to TUM format\n
        """)

    match command:
        case "1":
            build_ros_image(output_format())
        case "2":
            bin_merge_command()
        case "3":
            evo_traj()
        case "4":
            tum_conv()
        case _:
            print("Invalid choice. Exiting!")