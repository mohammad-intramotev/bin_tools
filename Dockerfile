ARG ROS_DISTRO
FROM ros:${ROS_DISTRO}-ros-base

# Make ROS_DISTRO available as an environment variable at container‐runtime
# ENV ROS_DISTRO=${ROS_DISTRO}

RUN apt-get update || true && \
    apt-get install -y python3-pip && \
    rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir numpy protobuf

#If we're on ROS2 (humble), install tf-transformations
RUN if [ "$ROS_DISTRO" = "humble" ]; then \
      apt-get update && \
      apt-get install -y ros-humble-tf-transformations && \
      rm -rf /var/lib/apt/lists/*; \
    fi

# Set working directory
WORKDIR /root/bintobag/build

# Copy project files into the container
COPY src/ .

# Auto‐source the ROS setup
RUN echo 'source /opt/ros/${ROS_DISTRO}/setup.bash' >> /root/.bashrc

# At runtime, invoke a single inline shell that checks $ROS_DISTRO
# and runs the correct Python script.
CMD ["bash", "-lc", "if [ \"$ROS_DISTRO\" = \"noetic\" ]; then \
                           python3 bin_to_ros1bag.py bin_files; \
                         else \
                           python3 bin_to_ros2bag.py bin_files; \
                         fi"]