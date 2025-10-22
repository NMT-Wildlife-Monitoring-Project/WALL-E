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

# Create the sgx group for GPIO access
RUN groupadd -g 993 gpio

# Add user to groups for hardware access, including sgx for GPIO
RUN usermod -a -G dialout,tty,audio,video,gpio $USER

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
    python3-serial \
    python3-venv \
    python3-full

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
    gpsd-tools

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
    ros-$ROS_DISTRO-joy \
    ros-$ROS_DISTRO-joy-linux

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

RUN apt-get install -y ros-$ROS_DISTRO-tf-transformations

# Install Python build dependencies for ROS
RUN apt-get install -y \
    python3-numpy \
    python3-rpi.gpio \
    python3-lgpio

# Install Adafruit libraries into the system Python
RUN pip3 install --break-system-packages adafruit-blinka adafruit-circuitpython-bno08x gps

# Install ROS robot localization and mapviz packages
RUN apt-get install -y \
    ros-$ROS_DISTRO-robot-localization

RUN apt-get install -y python3-rosdep

RUN apt-get install -y ros-$ROS_DISTRO-ros2-controllers
RUN apt-get install -y \
    ros-$ROS_DISTRO-robot-state-publisher \
    ros-$ROS_DISTRO-xacro
RUN apt-get install -y ros-$ROS_DISTRO-ros2-control

# RUN apt-get install -y \
#     ros-$ROS_DISTRO-mapviz \
#     ros-$ROS_DISTRO-mapviz-plugins \
#     ros-$ROS_DISTRO-tile-map \
#     ros-$ROS_DISTRO-multires-image

# Install rviz_imu_plugin for IMU visualization in RViz
RUN apt-get update && apt-get install -y ros-$ROS_DISTRO-rviz-imu-plugin

RUN apt-get install -y ros-$ROS_DISTRO-nav2-route

# Install ROS 2 command-line autocomplete
RUN apt-get update && apt-get install -y python3-argcomplete
RUN /bin/bash -c 'eval "$(register-python-argcomplete3 ros2)" >> /etc/bash.bashrc'

# Clean up
RUN rm -rf /var/lib/apt/lists/*

RUN usermod -a -G input $USER

# Copy the workspace into the container
COPY --chown=$USER:$USER ros2_ws /home/$USER/ros2_ws/
WORKDIR /home/$USER/ros2_ws

# Switch back to non-root user 'walle'
USER $USER
WORKDIR /home/$USER/ros2_ws

# Install rosdep dependencies as non-root user
# RUN rosdep update && \
#     rosdep install --from-paths /home/$USER/ros2_ws/src --ignore-src -r -y \
#     --skip-keys "mapviz mapviz_plugins tile_map nmea_msgs" || true

# Build the workspace
RUN /bin/bash -c '. /opt/ros/$ROS_DISTRO/setup.sh; \
                  export PATH="/opt/venv/bin:$PATH"; \
                  colcon build --symlink-install'

# Copy the entrypoint script into the container
USER root
WORKDIR /home/$USER
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Set entrypoint to use the external script
ENTRYPOINT ["/entrypoint.sh"]

# Ensure /dev/gpiochip* devices have correct permissions
RUN echo 'SUBSYSTEM=="gpio", KERNEL=="gpiochip*", MODE="0660", GROUP="sgx"' > /etc/udev/rules.d/99-gpio.rules

# Switch back to non-root user 'walle'
USER $USER
WORKDIR /home/$USER
