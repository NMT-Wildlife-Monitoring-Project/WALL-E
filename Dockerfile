# Argument for ROS version
ARG ROS_DISTRO=noetic

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

# Install stuff
# Docker will cache the layers, so if you change something in the Dockerfile,
# it will only rebuild the layers that have changed AND the layers that come after it.
# Avoid installing new stuff before large packages to avoid large rebuilds.
# Unless you have fast internet (pay to win smh)
RUN apt-get update && apt-get install -y python3
RUN apt-get install -y python3-pip
RUN apt-get install -y python3-serial
RUN apt-get install -y python3-catkin-tools
RUN apt-get install -y iputils-ping
RUN apt-get install -y ros-$ROS_DISTRO-navigation
RUN apt-get install -y ros-$ROS_DISTRO-teleop-twist-joy
RUN apt-get install -y ros-$ROS_DISTRO-joy
RUN apt-get install -y ros-$ROS_DISTRO-rviz
RUN apt-get install -y ros-$ROS_DISTRO-usb-cam
RUN apt-get install -y ros-$ROS_DISTRO-image-transport-plugins
RUN apt-get install -y ros-$ROS_DISTRO-image-view
RUN apt-get install -y avahi-daemon libnss-mdns avahi-utils
RUN apt-get install -y dbus

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
