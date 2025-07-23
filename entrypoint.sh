#!/bin/bash

set -e

source /opt/ros/${ROS_DISTRO}/setup.bash

case "$COMMAND" in
    1)
        python3 bin_merge.py
        ;;
    2|3|4)
        BUILD_WS="$(pwd)"
        ROS_WS="${BUILD_WS}/${ROS_VER}_ws"

        cd "${ROS_WS}"

        if [ "$ROS_DISTRO" = "noetic" ]; then
            catkin_make
            source "${ROS_WS}/devel/setup.bash"
        else
            colcon build
            source "${ROS_WS}/install/setup.bash"
        fi
        cd "${BUILD_WS}"

        case "$COMMAND" in
            2) python3 bin_to_ros1bagC.py ;;
            3) python3 bin_to_ros2bagC.py ;;
            4) python3 bin_to_ros2bagS.py ;;
        esac
        ;;
    5)
        python3 bin_to_tum.py
        ;;
    6)
        python3 tum_cmp.py
        ;;
    7)
        python3 bin_diagnose.py
        ;;
    *)
        echo "Unknown command: $COMMAND"
        ;;
esac