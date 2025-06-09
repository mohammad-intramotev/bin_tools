#!/bin/bash
set -e

source /opt/ros/${ROS_DISTRO}/setup.bash

if [ "$COMMAND" = "bin_merge" ]; then
    echo "Converting .bin to ROS bag"
    python3 bin_merge.py

elif [ "$COMMAND" = "bin_to_ros" ]; then
    if [ "$ROS_DISTRO" = "noetic" ]; then
        echo "Running ROS1 conversion"
        python3 bin_to_ros1bag.py
    else
        echo "Running ROS2 conversion"
        python3 bin_to_ros2bag.py
    fi

elif [ "$COMMAND" = "bin_to_tum" ]; then
    echo "Generating ground truth and converting to TUM"
    python3 bin_to_tum.py

elif [ "$COMMAND" = "traj_cmp" ]; then
    echo "Running trajectory comparison"
    python3 traj_cmp.py

else
    echo "Unknown command: $COMMAND"
fi
