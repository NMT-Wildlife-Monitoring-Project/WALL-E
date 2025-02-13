# Argument for ROS version
ARG ROS_DISTRO=noetic
ARG ARCH

# Base image with ROS on Ubuntu 20.04
FROM ros:$ROS_DISTRO-ros-base

# Set environment variables for ROS
ENV ROS_DISTRO=$ROS_DISTRO

# Create a non-root user named 'walle'
ARG USER=walle
ENV USER=$USER
RUN useradd -m $USER && \
    echo "$USER ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Set the user 'walle' as the default
USER $USER
WORKDIR /home/$USER

# Switch back to root
USER root

# Install necessary dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    wget \
    tar \
    python3 \
    python3-pip \
    python3-serial \
    python3-catkin-tools \
    iputils-ping \
    avahi-daemon \
    libnss-mdns \
    avahi-utils \
    dbus \
    zip \
    ros-$ROS_DISTRO-navigation \
    ros-$ROS_DISTRO-teleop-twist-joy \
    ros-$ROS_DISTRO-joy \
    ros-$ROS_DISTRO-rviz \
    ros-$ROS_DISTRO-usb-cam \
    ros-$ROS_DISTRO-image-transport-plugins \
    ros-$ROS_DISTRO-image-view

# Create catkin workspace
RUN mkdir -p /home/$USER/catkin_ws/src
RUN chown -R $USER:$USER /home/$USER/catkin_ws

# Download and extract the correct Slamtec SDK based on the architecture
WORKDIR /tmp
RUN ARCH=$(uname -m) && \
    if [ "$ARCH" = "aarch64" ]; then \
        wget -O sdk.tar.gz https://download-en.slamtec.com/api/download/slamware-ros-sdk_aarch64_gcc7/5.1.1-rtm?lang=netural && \
        tar -xzf sdk.tar.gz && \
        cp -r slamware_ros_sdk_linux-aarch64-gcc7/src/* /home/$USER/catkin_ws/src/ && \
        rm -rf /tmp/slamware_ros_sdk_linux-aarch64-gcc7 /tmp/sdk.tar.gz; \
    elif [ "$ARCH" = "x86_64" ]; then \
        wget -O sdk.tar.gz https://download-en.slamtec.com/api/download/slamware-ros-sdk_x86_64_gcc7/5.1.1-rtm?lang=netural && \
        tar -xzf sdk.tar.gz && \
        cp -r slamware_ros_sdk_linux-x86_64-gcc7/src/* /home/$USER/catkin_ws/src/ && \
        rm -rf /tmp/slamware_ros_sdk_linux-x86_64-gcc7 /tmp/sdk.tar.gz; \
    else \
        echo "Unsupported architecture: $ARCH" && exit 1; \
    fi

# Install pigpio library
RUN wget https://github.com/joan2937/pigpio/archive/master.zip && \
    unzip master.zip && \
    cd pigpio-master && \
    make && \
    make install && \
    rm -rf /tmp/pigpio-master /tmp/master.zip

WORKDIR /root

# Clean up
RUN rm -rf /var/lib/apt/lists/*

# Ensure the /var/run/dbus directory exists
RUN mkdir -p /var/run/dbus && chmod 755 /var/run/dbus

# Start dbus-daemon and avahi-daemon
RUN dbus-daemon --system --fork && avahi-daemon --daemonize

# Copy the ros2_ws folder into the container
COPY --chown=$USER:$USER catkin_ws /home/$USER/catkin_ws

USER $USER
WORKDIR /home/$USER/catkin_ws

# Build the workspace
USER $USER
RUN /bin/bash -c '. /opt/ros/$ROS_DISTRO/setup.sh; cd /home/$USER/catkin_ws; catkin build'

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
