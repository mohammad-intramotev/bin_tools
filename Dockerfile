ARG ROS_DISTRO
FROM ros:${ROS_DISTRO}-ros-base

RUN apt-get update || true && \
    apt-get install -y python3-pip && \
    rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir numpy protobuf

RUN if [ "$ROS_DISTRO" = "humble" ]; then \
      apt-get update && \
      apt-get install -y ros-humble-tf-transformations && \
      rm -rf /var/lib/apt/lists/*; \
    fi

# Set working directory
WORKDIR /root/bintobag/build

# Copy project files into the container
COPY src/ .

# Set up your environment
RUN echo 'source /opt/ros/${ROS_DISTRO}/setup.bash' >> /root/.bashrc

# Default command
# CMD ["python3", "bin_to_rosbag.py", "bin_files"]
# CMD ["tail", "-f", "/dev/null"]