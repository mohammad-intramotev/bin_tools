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
Modify `config.py` to your needs. This is the central configuration file used for defaults (e.g., file paths, ROS topics, GPS settings)

### 3. Run tools
```bash
python3 launch.py
```

You will be prompted to select a tool:

1. **Merge** multiple .bin files
2. **Convert** .bin to ROS bag (ROS1/ROS2)
3. **Export** ground-truth to TUM from .bin
4. **Compare** trajectories

Clean up with:  

```bash
sudo rm -rf output_files/*
```

## Tools Overview

### `combine_bin.py` — Merge Multiple `.bin` Files

Combines several `.bin` files into a single output file. This is useful when logs are split across multiple files from the same data collection session.

### `bin_to_ros1bag.py` / `bin_to_ros2bag.py` — Convert `.bin` to ROS Bag

Translates `.bin` files into ROS1 or ROS2 bag files.

### `bin_to_tum.py` — Export to TUM Format

Extracts GPS-based ground-truth data from `.bin` logs and converts it to the widely-used [TUM trajectory format](https://vision.in.tum.de/data/datasets/rgbd-dataset) for use with EVO, and evaluation tool.

### `traj_cmp.py` — Compare Trajectories

Takes two trajectory files (e.g., estimated vs ground truth) and computes error metrics such as RMSE, ATE, and relative pose error.