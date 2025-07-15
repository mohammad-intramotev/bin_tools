ARG ROS_DISTRO
FROM ros:${ROS_DISTRO}-ros-core

ARG ROS_DISTRO
ARG ROS_VER
RUN echo "---> Building for ROS Distro: $ROS_DISTRO"
RUN echo "---> ROS Workspace is set to: $ROS_VER"

# Set the environment to non-interactive
ENV DEBIAN_FRONTEND=noninteractive

# Install system dependencies
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        cmake \
        build-essential \
        python3-pip && \
    if [ "$ROS_DISTRO" = "humble" ]; then \
        apt-get install -y --no-install-recommends \
            ros-humble-ros2bag \
            ros-humble-rosbag2-py \
            ros-humble-sensor-msgs \
            ros-humble-rosbag2-storage-default-plugins \
            python3-colcon-common-extensions; \
    fi \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages
RUN pip3 install --no-cache-dir numpy protobuf pymap3d

# Create the ROS workspace structure
WORKDIR /root/bin_tools/build/${ROS_VER}_ws/src

# Copy the custom ROS 1 message package
COPY livox_custom_msgs_${ROS_VER}/ /root/bin_tools/build/${ROS_VER}_ws/src/livox_custom_msgs_${ROS_VER}/

# Set main working directory
WORKDIR /root/bin_tools/build

# Copy your application source code
COPY src/ .

# Copy and set permissions for the entrypoint script
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Set the entrypoint
ENTRYPOINT ["/entrypoint.sh"]

# Optional command for development to keep the container running
# CMD ["tail", "-f", "/dev/null"]