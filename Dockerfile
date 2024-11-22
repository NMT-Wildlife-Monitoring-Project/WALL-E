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

# Install necessary packages including catkin tools
USER root
RUN apt-get update && apt-get install -y \
    python3 \
    python3-pip \
    python3-serial \
    python3-catkin-tools \
    iputils-ping \
    ros-noetic-teleop-twist-joy \
    ros-noetic-joy \
    ros-noetic-navigation \
    ros-noetic-rviz \
    ros-noetic-usb-cam \
    && rm -rf /var/lib/apt/lists/*

# Copy the ros2_ws folder into the container
COPY --chown=$USER:$USER catkin_ws /home/$USER/catkin_ws

# Build the workspace
USER $USER
WORKDIR /home/$USER/catkin_ws
RUN /bin/bash -c '. /opt/ros/noetic/setup.sh; cd /home/$USER/catkin_ws; catkin build'

# Copy the entrypoint script into the container
USER root
WORKDIR /home/$USER
COPY entrypoint.sh /entrypoint.sh

# Make the entrypoint script executable
RUN chmod +x /entrypoint.sh

# Set entrypoint to use the external script
ENTRYPOINT ["/entrypoint.sh"]

# Switch back to non-root user 'walle'
USER $USER
WORKDIR /home/$USER
