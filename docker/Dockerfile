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

# Create hardware access groups
RUN groupadd -g 993 gpio && \
    groupadd -g 116 i2c && \
    usermod -a -G dialout,tty,audio,video,gpio,i2c,input $USER

# Switch to root for installations
USER root

# ============================================================================
# STAGE 1: Core System Dependencies
# ============================================================================
RUN apt-get update && apt-get install -y --no-install-recommends \
    software-properties-common \
    build-essential \
    cmake \
    wget \
    curl \
    tar \
    git \
    ca-certificates

# ============================================================================
# STAGE 2: Python and Build Tools
# ============================================================================
# Note: Avoid python3-full and python3.12-venv on ARM64 Jetson due to 404 errors
RUN apt-get install -y --no-install-recommends \
    python3 \
    python3-pip \
    python3-serial \
    && apt-get install -y --fix-missing \
    python3-venv

# ============================================================================
# STAGE 3: Networking and Utilities
# ============================================================================
RUN apt-get install -y --no-install-recommends \
    iputils-ping \
    avahi-daemon \
    libnss-mdns \
    avahi-utils \
    dbus \
    zip \
    net-tools

# ============================================================================
# STAGE 4: GPS and Hardware Support
# ============================================================================
RUN apt-get install -y --no-install-recommends \
    gpsd \
    gpsd-clients \
    gpsd-tools \
    libudev-dev \
    libsdl2-dev \
    joystick \
    python3-rpi.gpio \
    python3-lgpio

# ============================================================================
# STAGE 5: ROS 2 Core Packages
# ============================================================================
RUN apt-get install -y --no-install-recommends \
    ros-$ROS_DISTRO-rviz2 \
    ros-$ROS_DISTRO-navigation2 \
    ros-$ROS_DISTRO-nav2-bringup \
    ros-$ROS_DISTRO-slam-toolbox \
    ros-$ROS_DISTRO-robot-localization \
    ros-$ROS_DISTRO-tf2-tools

# ============================================================================
# STAGE 6: ROS 2 Control and Teleop
# ============================================================================
RUN apt-get install -y --no-install-recommends \
    ros-$ROS_DISTRO-teleop-twist-keyboard \
    ros-$ROS_DISTRO-teleop-twist-joy \
    ros-$ROS_DISTRO-joy \
    ros-$ROS_DISTRO-joy-linux \
    ros-$ROS_DISTRO-ros2-controllers \
    ros-$ROS_DISTRO-ros2-control \
    ros-$ROS_DISTRO-twist-mux

# ============================================================================
# STAGE 7: ROS 2 Communication and Transform
# ============================================================================
RUN apt-get install -y --no-install-recommends \
    ros-$ROS_DISTRO-rosbridge-suite \
    ros-$ROS_DISTRO-tf2-geometry-msgs \
    ros-$ROS_DISTRO-tf2-ros \
    ros-$ROS_DISTRO-tf-transformations \
    ros-$ROS_DISTRO-geometry-msgs \
    ros-$ROS_DISTRO-sensor-msgs \
    ros-$ROS_DISTRO-nav-msgs

# ============================================================================
# STAGE 8: ROS 2 Development
# ============================================================================
RUN apt-get install -y --no-install-recommends \
    ros-$ROS_DISTRO-rclcpp \
    ros-$ROS_DISTRO-rclcpp-components \
    ros-$ROS_DISTRO-std-msgs \
    ros-$ROS_DISTRO-visualization-msgs \
    ros-$ROS_DISTRO-robot-state-publisher \
    ros-$ROS_DISTRO-xacro \
    ros-$ROS_DISTRO-rviz-imu-plugin \
    ros-$ROS_DISTRO-nav2-route \
    python3-rosdep \
    python3-argcomplete

# ============================================================================
# STAGE 9: Command-line Autocomplete Setup
# ============================================================================
RUN /bin/bash -c 'eval "$(register-python-argcomplete3 ros2)" >> /etc/bash.bashrc'

# ============================================================================
# STAGE 10: Node.js for Web Frontend
# ============================================================================
RUN curl -fsSL https://deb.nodesource.com/setup_20.x | bash - && \
    apt-get install -y --no-install-recommends nodejs

# ============================================================================
# STAGE 11: Python Packages (Core and Web)
# ============================================================================
# Install numpy first with --ignore-installed to avoid Debian package conflicts
RUN pip3 install --break-system-packages --ignore-installed numpy

# Install Adafruit libraries
RUN pip3 install --break-system-packages \
    adafruit-blinka \
    adafruit-circuitpython-bno08x \
    adafruit-extended-bus \
    gps

# Install web app backend dependencies
RUN pip3 install --break-system-packages \
    fastapi \
    uvicorn[standard] \
    websockets \
    opencv-python-headless \
    PyYAML \
    python-multipart \
    httpx \
    aiofiles

# ============================================================================
# STAGE 12: Cleanup
# ============================================================================
RUN apt-get clean && \
    rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# ============================================================================
# User and Directory Setup
# ============================================================================
RUN mkdir -p /home/$USER && chown -R $USER:$USER /home/$USER && \
    mkdir -p /home/$USER/ros2_ws/log && chown -R $USER:$USER /home/$USER/ros2_ws/log

WORKDIR /home/$USER/ros2_ws

# ============================================================================
# Copy Source Code and Configuration
# ============================================================================
COPY --chown=$USER:$USER rviz2 /home/$USER/.rviz2
COPY --chown=$USER:$USER ros2_ws /home/$USER/ros2_ws/
COPY --chown=$USER:$USER web_app /home/$USER/web_app/

# ============================================================================
# Build ROS 2 Workspace
# ============================================================================
USER $USER
WORKDIR /home/$USER/ros2_ws
RUN /bin/bash -c '. /opt/ros/$ROS_DISTRO/setup.sh; \
                  export PATH="/opt/venv/bin:$PATH"; \
                  export COLCON_LOG_BASE="/home/$USER/ros2_ws/log"; \
                  colcon build --symlink-install'

# ============================================================================
# Build React Frontend
# ============================================================================
WORKDIR /home/$USER/web_app/frontend
RUN npm install && npm run build

# ============================================================================
# Setup Entrypoint
# ============================================================================
USER root
WORKDIR /home/$USER
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh && \
    echo 'SUBSYSTEM=="gpio", KERNEL=="gpiochip*", MODE="0660", GROUP="sgx"' > /etc/udev/rules.d/99-gpio.rules

ENTRYPOINT ["/entrypoint.sh"]

# ============================================================================
# Final Setup
# ============================================================================
USER $USER
WORKDIR /home/$USER
