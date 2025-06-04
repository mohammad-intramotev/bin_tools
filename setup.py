import os
import subprocess

def output_format():
    selection = input("Please enter output format (1 for ROS 1, 2 for ROS 2): ")

    while selection not in ('1', '2'):
        selection = input("Invalid input. Please enter output format (1 for ROS 1, 2 for ROS 2): ")

    return "noetic" if selection == '1' else "humble"

def build_ros_image(selection):
    env = {**os.environ, "ROS_DISTRO": selection}
 
    try:
        subprocess.run(["docker", "compose", "up", "--build"], env=env)
        print(f"Successfully built distro: {selection}")
    except subprocess.CalledProcessError:
        print(f"Failed to build distro: {selection}")
        
if __name__ == "__main__":
    selection = output_format()
    build_ros_image(selection)