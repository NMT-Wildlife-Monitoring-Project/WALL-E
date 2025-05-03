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

# Switch to root for installations
USER root

# Install necessary dependencies
# Install add-apt-repository support
RUN apt-get update \
 && apt-get install -y software-properties-common

# Add GCC7 PPA and install GCC–G++
RUN add-apt-repository ppa:ubuntu-toolchain-r/test -y \
 && apt-get update \
 && apt-get install -y gcc-7 g++-7 \
 && update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-7 50 \
 && update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-7 50

# Install build tools and utilities
RUN apt-get install -y build-essential cmake wget tar

# Download and extract the correct Slamtec SDK based on the architecture
WORKDIR /tmp
RUN bash -c "if [[ \$(uname -m) = \"aarch64\" || \$(uname -m) = \"x86_64\" ]]; then \
        wget -O sdk.tar.gz https://download-en.slamtec.com/api/download/slamware-ros-sdk_\$(uname -m)_gcc7/5.1.1-rtm?lang=netural && \
        tar -xzf sdk.tar.gz; \
    else \
        echo \"Unsupported architecture: \$(uname -m)\" && exit 73; \
    fi" && \
    mkdir -p /home/$USER/catkin_ws/src && \
    cp -r slamware_ros_sdk_linux-$(uname -m)-gcc7/src/* /home/$USER/catkin_ws/src/ && \
    rm -rf /tmp/slamware_* /tmp/sdk.tar.gz

# Install Python and serial support
RUN apt-get install -y python3 python3-pip python-pip python3-serial

# Install networking and mDNS
RUN apt-get install -y iputils-ping avahi-daemon libnss-mdns avahi-utils dbus zip

# Install ROS navigation & vision packages
RUN apt-get install -y \
    ros-$ROS_DISTRO-navigation \
    ros-$ROS_DISTRO-teleop-twist-joy \
    ros-$ROS_DISTRO-joy \
    ros-$ROS_DISTRO-rviz \
    ros-$ROS_DISTRO-usb-cam \
    ros-$ROS_DISTRO-image-transport-plugins \
    ros-$ROS_DISTRO-image-view

# Install GPS tooling
RUN apt-get install -y gpsd gpsd-clients python-gps

# Clean up
RUN rm -rf /var/lib/apt/lists/*



# Install Python dependencies
RUN pip3 install --upgrade pip && \
    pip3 install --no-cache-dir \
    flask \
    gps \
    opencv-python-headless \
    pigpio

RUN pip install --upgrade pip && \
    pip install --upgrade setuptools && \
    pip install --upgrade wheel && \
    pip install pigpio

# Initialize and build the catkin workspace
WORKDIR /home/$USER/catkin_ws/src
RUN /bin/bash -c '. /opt/ros/$ROS_DISTRO/setup.sh; catkin_init_workspace'

WORKDIR /home/$USER/catkin_ws
RUN /bin/bash -c '. /opt/ros/$ROS_DISTRO/setup.sh; catkin_make -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_COMPILER=/usr/bin/gcc-7 -DCMAKE_CXX_COMPILER=/usr/bin/g++-7 -DCMAKE_C_FLAGS="-w" -DCMAKE_CXX_FLAGS="-w"'

# Copy the workspace into the container
COPY --chown=$USER:$USER catkin_ws /home/$USER/catkin_ws/
RUN /bin/bash -c '. /opt/ros/$ROS_DISTRO/setup.sh; catkin_make'

# Copy the Flask app into the container
WORKDIR /home/$USER
COPY --chown=$USER:$USER web_app /home/$USER/web_app
RUN chmod +x /home/$USER/web_app/app.py

# Expose Flask app port
EXPOSE 5000

# Copy the entrypoint script into the container
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Set entrypoint to use the external script
ENTRYPOINT ["/entrypoint.sh"]

# Switch back to non-root user 'walle'
USER $USER
WORKDIR /home/$USER
