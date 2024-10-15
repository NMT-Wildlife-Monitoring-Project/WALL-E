# Base image with ROS Noetic on Ubuntu 20.04
FROM ros:noetic-ros-base

# Set environment variables for ROS
ENV ROS_DISTRO=noetic

# Create a non-root user named 'walle'
ARG USER=walle
RUN useradd -m $USER && \
    echo "$USER ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Set the user 'walle' as the default
USER $USER
WORKDIR /home/$USER

# Install necessary packages
USER root
RUN apt-get update && apt-get install -y \
    python3 \
    python3-pip \
    python3-serial \
    ros-noetic-teleop-twist-joy \
    ros-noetic-joy \
    && rm -rf /var/lib/apt/lists/*

# Switch back to non-root user 'walle'
USER $USER
WORKDIR /home/$USER

# Copy any local files needed (optional)
# COPY . .

# Set up entrypoint
ENTRYPOINT ["bash"]
