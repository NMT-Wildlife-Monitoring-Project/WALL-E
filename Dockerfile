# Argument for ROS version
ARG ROS_DISTRO=kinetic

# Base image with ROS on Ubuntu 20.04
FROM ros:$ROS_DISTRO-robot

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
    iputils-ping \
    avahi-daemon \
    libnss-mdns \
    avahi-utils \
    dbus \
    zip \
    software-properties-common \
    ros-$ROS_DISTRO-navigation \
    ros-$ROS_DISTRO-teleop-twist-joy \
    ros-$ROS_DISTRO-joy \
    ros-$ROS_DISTRO-rviz \
    ros-$ROS_DISTRO-usb-cam \
    ros-$ROS_DISTRO-image-transport-plugins \
    ros-$ROS_DISTRO-image-view

# Install GCC 7
RUN add-apt-repository ppa:ubuntu-toolchain-r/test -y && \
    apt-get update && \
    apt-get install -y gcc-7 g++-7

# Update alternatives to use GCC 7
RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-7 50 && \
    update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-7 50

# Install flask dependencies
# RUN pip3 install --upgrade pip && \
#     pip3 install flask
# RUN pip3 install opencv-python && \
#     pip3 install opencv-python-headless
# RUN apt-get install -y gpsd gpsd-clients python-gps

# Clean up
RUN rm -rf /var/lib/apt/lists/*

USER $USER

# Download and extract the correct Slamtec SDK based on the architecture
WORKDIR /tmp
RUN bash -c "if [[ \$(uname -m) = \"aarch64\" || \$(uname -m) = \"x86_64\" ]]; then \
        wget -O sdk.tar.gz https://download-en.slamtec.com/api/download/slamware-ros-sdk_\$(uname -m)_gcc7/5.1.1-rtm?lang=netural && \
        tar -xzf sdk.tar.gz; \
    else \
        echo \"Unsupported architecture: \$ARCH\" && exit 73; \
    fi"

# Install pigpio library
# USER root
# RUN wget https://github.com/joan2937/pigpio/archive/master.zip && \
#     unzip master.zip && \
#     cd pigpio-master && \
#     make && \
#     make install && \
#     rm -rf /tmp/pigpio-master /tmp/master.zip

# Install gpiozero library
RUN apt install python3-gpiozero

# Create catkin workspace
USER $USER
RUN mkdir -p /home/$USER/catkin_ws/src

RUN ARCH=$(uname -m) && \
    cp -r slamware_ros_sdk_linux-$ARCH-gcc7/src/* /home/$USER/catkin_ws/src/ && \
    rm -rf /tmp/slamware_* /tmp/sdk.tar.gz;

# Initialize the workspace
WORKDIR /home/$USER/catkin_ws/src
RUN /bin/bash -c '. /opt/ros/$ROS_DISTRO/setup.sh; catkin_init_workspace'

# Build the workspace
WORKDIR /home/$USER/catkin_ws
RUN /bin/bash -c '. /opt/ros/$ROS_DISTRO/setup.sh; catkin_make -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_COMPILER=/usr/bin/gcc-7 -DCMAKE_CXX_COMPILER=/usr/bin/g++-7 -DCMAKE_C_FLAGS="-w" -DCMAKE_CXX_FLAGS="-w"'

COPY --chown=$USER:$USER catkin_ws /home/$USER/catkin_ws/
RUN /bin/bash -c '. /opt/ros/$ROS_DISTRO/setup.sh; catkin_make'

# Copy the flask app into the container
# WORKDIR /home/$USER
# COPY --chown=$USER:$USER web_app /home/$USER/web_app
# RUN chmod +x /home/$USER/web_app/app.py
# EXPOSE 5000

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
