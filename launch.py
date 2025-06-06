import os
import subprocess

from src import bin_merge, traj_cmp

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


if __name__ == "__main__":
    command = input("""
        Welcome to bin_tools. Please enter a command:\n
        1) Convert .bin to ROS bag
        2) Combine multiple .bin files
        3) Compare trajectories\n
        """)
    
    # Input path for commands
    raw = input("\nPlease enter input path (or leave blank to use 'input_files/'): ").strip()
    input_path = os.path.expanduser(raw or "input_files")

    match command:
        case "1":
            build_ros_image(output_format())
        case "2":
            bin_merge.merge(input_path)
        case "3":
            pass
        case _:
            pass