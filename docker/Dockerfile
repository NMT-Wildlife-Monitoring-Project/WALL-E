# Argument for ROS version
ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}

# Set environment variables for ROS
ENV ROS_DISTRO=$ROS_DISTRO

# Create a non-root user named 'walle'
ARG USER=walle
ENV USER=$USER
RUN useradd -m $USER && \
    echo "$USER ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Add user to groups for hardware access
RUN usermod -a -G dialout,tty,audio,video $USER

# Switch to root for installations
USER root

# Install necessary dependencies
# Install add-apt-repository support
RUN apt-get update \
 && apt-get install -y software-properties-common

# Install build tools and utilities
RUN apt-get install -y \
    build-essential \
    cmake \
    wget \
    tar \
    git

# Install Python and serial support
RUN apt-get install -y \
    python3 \
    python3-pip \
    python3-serial

# Install networking and mDNS
RUN apt-get install -y \
    iputils-ping \
    avahi-daemon \
    libnss-mdns \
    avahi-utils \
    dbus \
    zip

# Install GPS tooling
RUN apt-get install -y \
    gpsd \
    gpsd-clients \
    gpsd-tools \
    python3-gps

# Install python dependencies
RUN apt-get install -y python3-numpy

# Install ROS navigation packages
RUN apt-get install -y \
    ros-$ROS_DISTRO-rviz2 \
    ros-$ROS_DISTRO-navigation2 \
    ros-$ROS_DISTRO-nav2-bringup

# Install ROS SLAM packages
RUN apt-get install -y \
    ros-$ROS_DISTRO-slam-toolbox

# Install ROS teleop packages
RUN apt-get install -y \
    ros-$ROS_DISTRO-teleop-twist-keyboard \
    ros-$ROS_DISTRO-teleop-twist-joy \
    ros-$ROS_DISTRO-joy

# Install ROS communication and transform packages
RUN apt-get install -y \
    ros-$ROS_DISTRO-rosbridge-suite \
    ros-$ROS_DISTRO-tf2-geometry-msgs \
    ros-$ROS_DISTRO-tf2-ros \
    ros-$ROS_DISTRO-geometry-msgs \
    ros-$ROS_DISTRO-sensor-msgs \
    ros-$ROS_DISTRO-nav-msgs

# Additional dependencies for Slamware SDK
RUN apt-get install -y \
    ros-$ROS_DISTRO-rclcpp \
    ros-$ROS_DISTRO-rclcpp-components \
    ros-$ROS_DISTRO-std-msgs \
    ros-$ROS_DISTRO-visualization-msgs

RUN apt-get update && apt-get install -y libudev-dev libsdl2-dev joystick

# Clean up
RUN rm -rf /var/lib/apt/lists/*

# Copy and build the roboclaw workspace
COPY --chown=$USER:$USER ros2_roboclaw_driver/ /home/$USER/ros2_roboclaw_driver/
WORKDIR /home/$USER/ros2_roboclaw_driver
RUN rosdep install --from-paths src --ignore-src -r -y
RUN /bin/bash -c '. /opt/ros/$ROS_DISTRO/setup.sh; colcon build --symlink-install'

# Copy the workspace into the container
COPY --chown=$USER:$USER ros2_ws /home/$USER/ros2_ws/
WORKDIR /home/$USER/ros2_ws
RUN /bin/bash -c '. /opt/ros/$ROS_DISTRO/setup.sh; \
                  colcon build --symlink-install'

WORKDIR /home/$USER
# Copy the entrypoint script into the container
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Set entrypoint to use the external script
ENTRYPOINT ["/entrypoint.sh"]

# Switch back to non-root user 'walle'
USER $USER
WORKDIR /home/$USER
