# bin_tools
`bin_tools` provides a suite of utilities for processing `.bin` log files.

## Prerequisites
- [Docker](https://docs.docker.com/engine/install/)
- Python 3

## Setup
### 1. Clone repository
```bash
git clone https://github.com/mohammad-intramotev/bin_tools.git
cd bin_tools
```

### 2. Edit configuration
Modify `config.py` to your needs. This is the central configuration file used for defaults (e.g., file paths, ROS topics, channel names).

### 3. Run tools
```bash
python3 launch.py
```

You will be prompted to select a tool:

1. **Merge** multiple .bin files
2. **Convert** .bin to ROS 1 bag with custom messages
3. **Convert** .bin to ROS 2 bag with custom messages
4. **Convert** .bin to ROS 2 bag with standard messages
5. **Export** ground-truth to TUM from .bin
6. **Compare** trajectories
7. **Diagnose** .bin file

Clean up with:  

```bash
sudo rm -rf output_files/*
```

## Tools Overview

### `bin_merge.py` — Merge Multiple `.bin` Files
Combines several `.bin` logs into a single output file. This is useful when logs are split across multiple files from the same data collection session.

### `bin_to_ros1bagC.py` — Convert `.bin` to ROS 1 Bag
Converts `.bin` files into ROS 1 bags containing LiDAR and IMU data with custom messages.

### `bin_to_ros2bagC.py` — Convert `.bin` to ROS 2 Bag
Converts `.bin` files into ROS 2 bags containing LiDAR and IMU data with custom messages.

### `bin_to_ros2bagS.py` — Convert `.bin` to ROS 2 Bag
Converts `.bin` files into ROS 2 bags containing LiDAR and IMU data with standard messages.

### `bin_to_tum.py` — Export to TUM Format
Extracts GPS-based ground-truth data from `.bin` logs and converts it to the widely-used [TUM trajectory format](https://vision.in.tum.de/data/datasets/rgbd-dataset) for use with EVO, and evaluation tool.

### `traj_cmp.py` — Compare Trajectories
Takes two trajectory files (e.g., estimated vs ground truth) and computes error metrics such as RMSE, ATE, and relative pose error.

### `bin_diagnose.py` — Diagnose .bin Logs
Parses `.bin` logs to report data loss and summarize key metrics.
