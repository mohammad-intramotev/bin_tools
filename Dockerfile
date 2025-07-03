# Use ROS Noetic as the base image
FROM ros:noetic-ros-core

# Set the environment to non-interactive to prevent prompts during build
ENV DEBIAN_FRONTEND=noninteractive

# Install system dependencies.
# Explicitly remove the system's python3-protobuf to avoid conflicts with the pip version.
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        git \
        cmake \
        build-essential \
        python3-pip \
        ros-noetic-pcl-ros \
    && apt-get remove -y python3-protobuf && \
    rm -rf /var/lib/apt/lists/*

# Install Python packages using pip
RUN pip3 install --no-cache-dir \
    "numpy<1.24" \
    "protobuf<=3.20.3" \
    pymap3d \
    evo

# Set working directory for subsequent commands
WORKDIR /root/bin_tools/build

# Copy your application source code into the container
COPY src/ .

# Copy and set permissions for the entrypoint script
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Set the entrypoint to run when the container starts
ENTRYPOINT ["/entrypoint.sh"]

# Optional command for development to keep the container running
# CMD ["tail", "-f", "/dev/null"]
