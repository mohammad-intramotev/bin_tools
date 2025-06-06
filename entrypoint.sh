#!/bin/bash
set -e

source /opt/ros/${ROS_DISTRO}/setup.bash

if [ "$COMMAND" = "bin_convert" ]; then
    if [ "$ROS_DISTRO" = "noetic" ]; then
        echo "Running ROS1 conversion"
        python3 bin_to_ros1bag.py bin_files
    else
        echo "Running ROS2 conversion"
        python3 bin_to_ros2bag.py bin_files
    fi

elif [ "$COMMAND" = "traj_cmp" ]; then
    echo "Running trajectory comparison"
    python3 traj_comp.py

else
    echo "Unknown command: $COMMAND"
    exec "$@"
fi
