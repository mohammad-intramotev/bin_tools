#!/bin/bash
set -e

source /opt/ros/noetic/setup.bash

BUILD_WS="$(pwd)"
SDK_WS="${BUILD_WS}/Livox-SDK"
LIVOX_WS="${BUILD_WS}/livox_ws"

if [ "$COMMAND" = "bin_merge" ]; then
    echo "Converting .bin to ROS bag"
    python3 bin_merge.py

elif [ "$COMMAND" = "bin_to_ros" ]; then
    # Step 1: Install Livox-SDK
    if [ ! -d "${SDK_WS}/.git" ]; then
        git clone https://github.com/Livox-SDK/Livox-SDK.git "${SDK_WS}"
    fi
    cmake -S "${SDK_WS}" -B "${SDK_WS}/build"
    cmake --build "${SDK_WS}/build" -- -j$(nproc)
    sudo cmake --install "${SDK_WS}/build"

    # Step 2: Build livox_ros_driver
    mkdir -p "${LIVOX_WS}/src"
    cd "${LIVOX_WS}/src"
    [ ! -d "livox_ros_driver/.git" ] && git clone https://github.com/Livox-SDK/livox_ros_driver.git
    cd "${LIVOX_WS}" && catkin_make

    # Step 3: Run application
    source "${LIVOX_WS}/devel/setup.bash"
    cd "${BUILD_WS}"
    python3 bin_to_ros1bag.py

elif [ "$COMMAND" = "bin_to_tum" ]; then
    echo "Generating ground truth and converting to TUM"
    python3 bin_to_tum.py

elif [ "$COMMAND" = "traj_cmp" ]; then
    echo "Running trajectory comparison"
    python3 traj_cmp.py

else
    echo "Unknown command: $COMMAND"
fi
