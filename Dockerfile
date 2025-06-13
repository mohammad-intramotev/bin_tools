FROM ros:noetic-ros-core

# Set to noninteractive
ENV DEBIAN_FRONTEND=noninteractive

# Fix ROS key issue and install core dependencies
RUN apt-get update || true && apt-get install -y --no-install-recommends \
    curl gnupg2 lsb-release ca-certificates \
    && rm -rf /var/lib/apt/lists/* \
    && rm -f /etc/apt/sources.list.d/ros1-latest.list \
             /usr/share/keyrings/ros1-latest-archive-keyring.gpg \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros1-latest-archive-keyring.gpg \
    && echo \
       "deb [signed-by=/usr/share/keyrings/ros1-latest-archive-keyring.gpg] \
       http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" \
       > /etc/apt/sources.list.d/ros-latest.list

# Install system dependencies
RUN apt-get update && \
    apt-get install -y \
      git \
      ros-noetic-pcl-ros \
    && apt-get remove -y python3-protobuf && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
      python3-pip && \
    rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir "numpy<1.24" protobuf pymap3d evo 

# Set working directory
WORKDIR /root/bin_tools/build

# Copy project files into the container
COPY src/ .

# Copy entrypoint script into the container
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Set the entrypoint
ENTRYPOINT ["/entrypoint.sh"]

# For development
# CMD ["tail", "-f", "/dev/null"]
