FROM ros:jazzy
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y python3-pip
ENV SHELL=/bin/bash

RUN apt-get install -y ros-$ROS_DISTRO-rviz2

CMD ["/bin/bash"]
