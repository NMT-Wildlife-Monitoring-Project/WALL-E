#!/bin/bash
set -e

# Default values
IMAGE_NAME="walle/ros1:noetic"
IP=""
ROS_MASTER_PORT=11311
DISPLAY_ENABLED=false
DOCKER_RUN_FLAGS=()
COMMAND_TO_RUN=""
ENV_FILE="env_file.txt"
RUN_ROSCORE=false

# Function to show usage
usage() {
    echo "Usage: $0 [--start | --teleop | --usb-cam | --command <command> | --roscore] [--port <port>] [--ip <host_ip>] [--display] [--help]"
    echo "  --start                     Run robot_start.launch from package control"
    echo "  --teleop                    Run teleop.launch from package control"
    echo "  --usb-cam                   Run rosrun usb_cam usb_cam_node"
    echo "  --command (-c) <command>    Pass a command to be run in the container"
    echo "  --roscore                   Run roscore"
    echo "  --port <port>               Specify custom ROS master port (default is 11311)"
    echo "  --ip <host_ip>              Specify host IP"
    echo "  --display                   Enable display support (forward X11 display)"
    echo "  --help (-h)                 Show this help message"
    exit 1
}

# Parse options
while [[ "$#" -gt 0 ]]; do
    case "$1" in
        --ip) IP="$2"; shift 2 ;;
        --start) RUN_ROBOT_LAUNCH=true; shift ;;
        --teleop) RUN_TELEOP_LAUNCH=true; shift ;;
        --usb-cam) RUN_USB_CAM_NODE=true; shift ;;
        --command) COMMAND_TO_RUN="$2"; shift 2 ;;
        --roscore) RUN_ROSCORE=true; shift ;;
        --port) ROS_MASTER_PORT="$2"; shift 2 ;;
        --display) DISPLAY_ENABLED=true; shift ;;
        --help) usage; shift ;;
        -c) COMMAND_TO_RUN="$2"; shift 2 ;;
        -h) usage; shift ;;
        *) usage; shift ;;
    esac
done

# Get the active IP address if not provided
if [[ -z "$IP" ]]; then
    IP=$(hostname -I | awk '{print $1}')
    if [[ -z "$IP" ]]; then
        echo "Error: Unable to determine the host IP address."
        exit 1
    fi
fi

# Set ROS_IP and ROS_MASTER_URI
ROS_IP="$IP"
ROS_MASTER_URI="http://raspberrypi:$ROS_MASTER_PORT"

# Write environment variables to file
echo "ROS_IP=$ROS_IP" > $ENV_FILE
echo "ROS_MASTER_URI=$ROS_MASTER_URI" >> $ENV_FILE

# No longer part of the Bourgeois 
DOCKER_RUN_FLAGS+=("--privileged")
DOCKER_RUN_FLAGS+=("--net=host")

# Add Docker flag to mount /dev with correct permissions
DOCKER_RUN_FLAGS+=("--volume=/dev:/dev:rw")

# Enable display if requested
if [[ "$DISPLAY_ENABLED" == true ]]; then
    echo "DISPLAY=$DISPLAY" >> $ENV_FILE
    DOCKER_RUN_FLAGS+=("--volume=/tmp/.X11-unix:/tmp/.X11-unix:rw")
    xhost +local:docker
fi

# Build the container
docker build -t $IMAGE_NAME .

# Check if the container is already running
if docker ps -q -f ancestor=$IMAGE_NAME | grep -q .; then
    echo "A container from image $IMAGE_NAME is already running."
else
    echo "Starting a new Docker container..."
    docker run -dit --env-file $ENV_FILE "${DOCKER_RUN_FLAGS[@]}" $IMAGE_NAME bash
    if [ "$RUN_ROSCORE" = true ]; then
        echo "Running roscore..."
        docker exec --env-file $ENV_FILE -it $(docker ps -q -f ancestor=$IMAGE_NAME) /entrypoint.sh roscore
    fi

fi

# Execute the specified option
if [ "$RUN_ROBOT_LAUNCH" = true ]; then
    echo "Running robot_start.launch..."
    docker exec --env-file $ENV_FILE -it $(docker ps -q -f ancestor=$IMAGE_NAME) /entrypoint.sh roslaunch control robot_start.launch
elif [ "$RUN_TELEOP_LAUNCH" = true ]; then
    echo "Running teleop.launch..."
    docker exec --env-file $ENV_FILE -it $(docker ps -q -f ancestor=$IMAGE_NAME) /entrypoint.sh roslaunch control teleop.launch
elif [ "$RUN_USB_CAM_NODE" = true ]; then
    echo "Running rosrun usb_cam usb_cam_node..."
    docker exec --env-file $ENV_FILE -it $(docker ps -q -f ancestor=$IMAGE_NAME) /entrypoint.sh rosrun usb_cam usb_cam_node
elif [ -n "$COMMAND_TO_RUN" ]; then
    echo "Running custom command: $COMMAND_TO_RUN"
    docker exec --env-file $ENV_FILE -it $(docker ps -q -f ancestor=$IMAGE_NAME) /entrypoint.sh $COMMAND_TO_RUN
else
    echo "No options specified. Opening interactive bash terminal in the container..."
    docker exec --env-file $ENV_FILE -it $(docker ps -q -f ancestor=$IMAGE_NAME) /entrypoint.sh bash
fi
