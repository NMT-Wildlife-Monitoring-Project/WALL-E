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

COPY --chown=$USER:$USER catkin_ws /home/$USER/catkin_ws

# Copy the entrypoint script into the container
COPY entrypoint.sh /entrypoint.sh

# Make the entrypoint script executable
RUN chmod +x /entrypoint.sh

# Set entrypoint to use the external script
ENTRYPOINT ["/entrypoint.sh"]

# Switch back to non-root user 'walle'
USER $USER
WORKDIR /home/$USER
