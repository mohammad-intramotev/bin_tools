FROM ros:noetic-ros-core

# Set to noninteractive
ENV DEBIAN_FRONTEND=noninteractive

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
