#!/bin/bash
set -e

source /opt/ros/${ROS_DISTRO}/setup.bash

if [ "$COMMAND" = "bin_merge" ]; then
    python3 bin_merge.py

elif [ "$COMMAND" = "bin_to_ros" ]; then
    BUILD_WS="$(pwd)"
    ROS_WS="${BUILD_WS}/${ROS_VER}_ws"

    cd "${ROS_WS}"

    if [ "$ROS_DISTRO" = "noetic" ]; then
        catkin_make
        source "${ROS_WS}/devel/setup.bash"
        cd "${BUILD_WS}"
        python3 bin_to_ros1bag.py
    else
        colcon build
        source "${ROS_WS}/install/setup.bash"
        cd "${BUILD_WS}"
        python3 bin_to_ros2bag.py
    fi

elif [ "$COMMAND" = "bin_to_tum" ]; then
    python3 bin_to_tum.py

elif [ "$COMMAND" = "traj_cmp" ]; then
    python3 traj_cmp.py

elif [ "$COMMAND" = "bin_diag" ]; then
    python3 bin_diagnose.py

else
    echo "Unknown command: $COMMAND"
fi
