# bin_tools
Various tools for our .bin log files.

## Setup
1. Place `.bin` file(s) into [`bin_files/`](./bin_files/).  
   (You can change this default location in the [`Dockerfile`](./Dockerfile).)
2. If needed, use [`combine_bin.py`](./src/combine_bin.py) to merge multiple .bin files into a single file.
3. To start the converter, first run `launch.py`:

   ```bash
   python3 launch.py
   ```
4. When prompted, enter the desired ROS version.
5. Generated bags will be placed in `ros_bags_output/ros(x)_bags/`,  
   where `x` is `1` or `2` depending on the ROS version you selected.
6. To delete all generated outputs:
   ```bash
   sudo rm -rf ros_bags_output/*
   ```