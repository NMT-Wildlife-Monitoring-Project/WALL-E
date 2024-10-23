FROM ros:humble
VOLUME /dev /dev
VOLUME /tmp/.X11-unix
ENV DISPLAY=0

RUN apt-get update && apt-get install -y sudo

# Create a non-root user named 'walle'
ARG USER=walle
RUN useradd -m $USER && \
    echo "$USER ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

USER $USER
WORKDIR /home/$USER

# Install system packages
USER root
RUN apt-get install -y python3
RUN apt-get install -y python3-pip

# Install python packages
RUN pip3 install pyserial

# Install ros packages
RUN apt-get -y install ros-humble-teleop-twist-joy
RUN apt-get -y install ros-humble-joy
RUN apt-get -y install ros-humble-navigation

# Copy in the ros workspace
COPY --chown=$USER:$USER ros2_ws /home/$USER/ros2_ws

# Compile the ros workspace
USER $USER
WORKDIR /home/$USER/ros2_ws
RUN /bin/bash -c '. /opt/ros/humble/setup.sh; cd /home/$USER/ros2_ws; colcon build'

# Copy in the entrypoint script
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

#sudo docker run --rm -it --env DISPLAY=$DISPLAY --volume /dev:/dev  -v /home/pi5-walle/WALL-E/:/WALL-E/    --volume /tmp/.X11-unix:/tmp/.X11-unix walle"
