FROM ros:humble
#VOLUME /dev /dev
#VOLUME /home/pi5-walle/WALL-E /WALL-E/
#VOLUME /tmp/.X11-unix
#ENV DISPLAY=0


RUN apt-get update && apt-get install -y sudo

# Create a non-root user and group (walle)
RUN groupadd -g 1001 walle && \
   useradd -u 1001 -g walle -m -s /bin/bash walle

# Grant sudo permissions to the user
RUN echo "walle ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Set the permissions for files or directories if needed
# For example, set permissions to a directory (e.g., /app)
RUN mkdir /app && chown -R walle:walle /app

# Switch to the non-root user
USER walle

# Set the working directory (optional)
#WORKDIR /app
RUN sudo apt-get install -y python3
RUN sudo apt-get install -y python3-pip

# Run commands as the new user (install pySerial)
RUN pip3 install pyserial
RUN sudo apt-get -y install ros-humble-teleop-twist-joy
RUN sudo apt-get -y install ros-humble-joy
RUN sudo apt-get install -y python3

CMD ["/bin/bash"]


#RUN cd /WALL-E/ros2_ws/src/motor_control/
#RUN colcon build --symlink-install   




#sudo docker run --rm -it --env DISPLAY=$DISPLAY --volume /dev:/dev  -v /home/pi5-walle/WALL-E/:/WALL-E/    --volume /tmp/.X11-unix:/tmp/.X11-unix walle"