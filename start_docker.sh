#!/bin/bash

# Default values
CONTAINER_NAME="walle_ros1"
IMAGE_NAME="walle/ros1:noetic"
# SLAVE_IP="129.138.167.168"
# HOST_IP="129.138.175.118"
SLAVE_IP="192.168.1.81"
HOST_IP="192.168.1.231"
ROS_MASTER_PORT=11311
MODE=""
DISPLAY_ENABLED=false
DOCKER_RUN_FLAGS=()
COMMAND_TO_RUN=""

# Function to show usage
usage() {
    echo "Usage: $0 [-h | -s] [-r | -t | -c <command>] [-p <port>] [-i <host_ip> -j <slave_ip>] [-d] [-x]"
    echo "  -h          Run on host"
    echo "  -s          Run on slave"
    echo "  -r          Run robot_start.launch from package control"
    echo "  -t          Run teleop.launch from package control"
    echo "  -c <command> Pass a command to be run in the container"
    echo "  -p <port>   Specify custom ROS master port (default is 11311)"
    echo "  -i <host_ip> Specify host IP"
    echo "  -j <slave_ip> Specify slave IP"
    echo "  -d          Enable display support (forward X11 display)"
    echo "  -x          Show this help message"
    exit 1
}

# Parse options
while getopts "hsi:j:rtcp:dx" opt; do
    case "$opt" in
        h) MODE="host" ;;
        s) MODE="slave" ;;
        i) HOST_IP="$OPTARG" ;;
        j) SLAVE_IP="$OPTARG" ;;
        r) RUN_ROBOT_LAUNCH=true ;;
        t) RUN_TELEOP_LAUNCH=true ;;
        c) COMMAND_TO_RUN="$OPTARG" ;;
        p) ROS_MASTER_PORT="$OPTARG" ;;
        d) DISPLAY_ENABLED=true ;;
        x) usage ;;
        *) usage ;;
    esac
done

# Validate mode
if [[ -z "$MODE" ]]; then
    echo "Error: You must specify -h for host or -s for slave."
    usage
fi

# Set ROS_IP and ROS_MASTER_URI based on mode
if [[ "$MODE" == "host" ]]; then
    ROS_IP="$HOST_IP"
    ROS_MASTER_URI="http://$HOST_IP:$ROS_MASTER_PORT"
elif [[ "$MODE" == "slave" ]]; then
    ROS_IP="$SLAVE_IP"
    ROS_MASTER_URI="http://$HOST_IP:$ROS_MASTER_PORT"
fi

# Add environment variables to Docker flags
DOCKER_RUN_FLAGS+=("-e ROS_IP=$ROS_IP")
DOCKER_RUN_FLAGS+=("-e ROS_MASTER_URI=$ROS_MASTER_URI")

# Mount /dev into the container
DOCKER_RUN_FLAGS+=("-v /dev:/dev")

# Enable display if requested
if [[ "$DISPLAY_ENABLED" == true ]]; then
    DOCKER_RUN_FLAGS+=("-e DISPLAY=$DISPLAY")
    DOCKER_RUN_FLAGS+=("-v /tmp/.X11-unix:/tmp/.X11-unix:rw")
    xhost +local:docker
fi

# Build the container image
echo "Building Docker image..."
docker build -t $IMAGE_NAME .

# Start the container if not already running
if [ ! "$(docker ps -q -f name=$CONTAINER_NAME)" ]; then
    if [ "$(docker ps -aq -f status=exited -f name=$CONTAINER_NAME)" ]; then
        # Remove the stopped container
        docker rm $CONTAINER_NAME
    fi
    echo "Starting Docker container..."
    docker run -dit --name $CONTAINER_NAME "${DOCKER_RUN_FLAGS[@]}" --privileged $IMAGE_NAME
else
    echo "Container is already running."
fi

# Execute the specified option
if [ "$RUN_ROBOT_LAUNCH" = true ]; then
    echo "Running robot_start.launch..."
    docker exec -it $CONTAINER_NAME bash -c "source /opt/ros/noetic/setup.bash && source ~/catkin_ws/devel/setup.bash && roslaunch control robot_start.launch"
elif [ "$RUN_TELEOP_LAUNCH" = true ]; then
    echo "Running teleop.launch..."
    docker exec -it $CONTAINER_NAME bash -c "source /opt/ros/noetic/setup.bash && source ~/catkin_ws/devel/setup.bash && roslaunch control teleop.launch"
elif [ -n "$COMMAND_TO_RUN" ]; then
    echo "Running custom command: $COMMAND_TO_RUN"
    docker exec -it $CONTAINER_NAME bash -c "source /opt/ros/noetic/setup.bash && source ~/catkin_ws/devel/setup.bash && $COMMAND_TO_RUN"
else
    echo "No options specified. Opening interactive bash terminal in the container..."
    docker exec -it $CONTAINER_NAME bash -c "source /opt/ros/noetic/setup.bash && source ~/catkin_ws/devel/setup.bash && bash"
fi
