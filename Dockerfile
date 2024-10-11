FROM ros:humble
#VOLUME /dev /dev
#VOLUME /home/pi5-walle/WALL-E /WALL-E/
#VOLUME /tmp/.X11-unix
#ENV DISPLAY=0


#RUN apt-get update && apt-get install -y sudo

# Create a non-root user and group (appuser)
#RUN groupadd -g 1001 appuser && \
#   useradd -u 1001 -g appuser -m -s /bin/bash appuser

# Grant sudo permissions to the user
#RUN echo "appuser ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Set the permissions for files or directories if needed
# For example, set permissions to a directory (e.g., /app)
#RUN mkdir /app && chown -R appuser:appuser /app

# Switch to the non-root user
USER appuser:sudo

# Set the working directory (optional)
#WORKDIR /app


CMD ["/bin/bash"]



#RUN useradd -ms /bin/bash walle

# Add the new user to the sudo group (for sudo privileges)
#RUN usermod -aG sudo walle

# Set permissions for the new user's home directory (if needed)
#RUN chown -R walle:walle /home/walle

# Switch to the new user
#USER walle

# Set the working directory to the new user's home
#WORKDIR /home/walle

#RUN apt-get install -y python3
# Run commands as the new user (install pySerial)
#RUN pip3 install --force-yes pyserial
#RUN sudo apt update
#RUN sudo apt-get -y install ros-humble-teleop-twist-joy
#RUN sudo apt-get -y install ros-humble-joy
#RUN sudo apt-get install -y python3
#RUN sudo pip3 install pyserial

#RUN cd /WALL-E/ros2_ws/src/motor_control/
#RUN colcon build --symlink-install   




#sudo docker run --rm -it --env DISPLAY=$DISPLAY --volume /dev:/dev  -v /home/pi5-walle/WALL-E/:/WALL-E/    --volume /tmp/.X11-unix:/tmp/.X11-unix ros:humble"
