ARG ROS_DISTRO
FROM ros:${ROS_DISTRO}-ros-base

RUN apt-get update || true && \
    apt-get install -y python3-pip && \
    rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir numpy protobuf evo

# If we're on ROS2 (humble), install tf-transformations
RUN if [ "$ROS_DISTRO" = "humble" ]; then \
      apt-get update && \
      apt-get install -y ros-humble-tf-transformations && \
      rm -rf /var/lib/apt/lists/*; \
    fi

# Set working directory
WORKDIR /root/bin_tools/build

# Copy project files into the container
COPY src/ .

# Copy entrypoint script into the container
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Set the entrypoint
ENTRYPOINT ["/entrypoint.sh"]