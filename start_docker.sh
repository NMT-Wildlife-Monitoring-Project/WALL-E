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
BUILD_CONTAINER=false
STOP_CONTAINER=false

# Function to show usage
usage() {
    echo "Usage: $0 [--start (-s) | --teleop (-t) | --usb-cam (-u) | --command (-c) <command> | --roscore (-r) | --build (-b) | --stop (-x)] [--port (-p) <port>] [--ip (-i) <host_ip>] [--display (-d)] [--help (-h)]"
    echo "  --start (-s)                Run robot_start.launch from package control"
    echo "  --teleop (-t)               Run teleop.launch from package control"
    echo "  --usb-cam (-u)              Run rosrun usb_cam usb_cam_node"
    echo "  --command (-c) <command>    Pass a command to be run in the container"
    echo "  --roscore (-r)              Run roscore"
    echo "  --port (-p) <port>          Specify custom ROS master port (default is 11311)"
    echo "  --ip (-i) <host_ip>         Specify host IP"
    echo "  --display (-d)              Enable display support (forward X11 display)"
    echo "  --build (-b)                Build the Docker container"
    echo "  --stop (-x)                 Stop the Docker container if it is running"
    echo "  --help (-h)                 Show this help message"
    exit 1
}

# Parse options
while [[ "$#" -gt 0 ]]; do
    case "$1" in
        --ip|-i) IP="$2"; shift 2 ;;
        --start|-s) RUN_ROBOT_LAUNCH=true; shift ;;
        --teleop|-t) RUN_TELEOP_LAUNCH=true; shift ;;
        --usb-cam|-u) RUN_USB_CAM_NODE=true; shift ;;
        --command|-c) COMMAND_TO_RUN="$2"; shift 2 ;;
        --roscore|-r) RUN_ROSCORE=true; shift ;;
        --port|-p) ROS_MASTER_PORT="$2"; shift 2 ;;
        --display|-d) DISPLAY_ENABLED=true; shift ;;
        --build|-b) BUILD_CONTAINER=true; shift ;;
        --stop|-x) STOP_CONTAINER=true; shift ;;
        --help|-h) usage; shift ;;
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
ROS_MASTER_URI="http://raspberrypi.local:$ROS_MASTER_PORT"

# Write environment variables to file
echo "ROS_IP=$ROS_IP" > $ENV_FILE
echo "ROS_MASTER_URI=$ROS_MASTER_URI" >> $ENV_FILE

# No longer part of the Bourgeois 
DOCKER_RUN_FLAGS+=("--privileged")
DOCKER_RUN_FLAGS+=("--net=host")

# Add Docker flag to mount /dev with correct permissions
DOCKER_RUN_FLAGS+=("--volume=/dev:/dev:rw")

if [ "$RUN_USB_CAM_NODE" = true ]; then
    if [ -e /dev/video0 ]; then
        DOCKER_RUN_FLAGS+=("--device=/dev/video0")
    else
        echo "Error: /dev/video0 does not exist."
        exit 1
    fi
fi

# Enable display if requested
if [[ "$DISPLAY_ENABLED" = true ]]; then
    echo "DISPLAY=$DISPLAY" >> $ENV_FILE
    DOCKER_RUN_FLAGS+=("--volume=/tmp/.X11-unix:/tmp/.X11-unix:rw")
    xhost +local:docker
fi

# Build the container if requested
if [[ "$BUILD_CONTAINER" = true ]]; then
    echo "Building the Docker container..."
    docker build -t $IMAGE_NAME .
fi

# Check if the container is already running
RUNNING=false
if docker ps -q -f ancestor=$IMAGE_NAME | grep -q .; then
    echo "Docker container is already running."
    RUNNING=true
else
    echo "Docker container is not running."
fi

# Stop the container if requested
if [[ "$STOP_CONTAINER" = true && "$RUNNING" = true ]]; then
    echo "Stopping the running Docker container..."
    docker stop $(docker ps -q -f ancestor=$IMAGE_NAME)
    RUNNING=false
fi

# Start the container if it is not already running
if [[ "$RUNNING" = false ]]; then
    echo "Starting the Docker container..."
    docker run -dit --env-file $ENV_FILE "${DOCKER_RUN_FLAGS[@]}" $IMAGE_NAME bash
fi

CONTAINER_ID=$(docker ps -q -f ancestor=$IMAGE_NAME)
echo "Container ID: $CONTAINER_ID"

if [ "$RUN_ROSCORE" = true ]; then
    echo "Running roscore..."
    docker exec --env-file $ENV_FILE -it $CONTAINER_ID /entrypoint.sh roscore
fi

# Execute the specified option
if [ "$RUN_ROBOT_LAUNCH" = true ]; then
    echo "Running robot_start.launch..."
    docker exec --env-file $ENV_FILE -it $CONTAINER_ID /entrypoint.sh roslaunch control robot_start.launch
elif [ "$RUN_TELEOP_LAUNCH" = true ]; then
    echo "Running teleop.launch..."
    docker exec --env-file $ENV_FILE -it $CONTAINER_ID /entrypoint.sh roslaunch control teleop.launch
elif [ "$RUN_USB_CAM_NODE" = true ]; then
    echo "Running rosrun usb_cam usb_cam_node with /dev/video0..."
    docker exec --env-file $ENV_FILE -it $CONTAINER_ID /entrypoint.sh roslaunch control usb_cam.launch
elif [ -n "$COMMAND_TO_RUN" ]; then
    echo "Running custom command: $COMMAND_TO_RUN"
    docker exec --env-file $ENV_FILE -it $CONTAINER_ID /entrypoint.sh $COMMAND_TO_RUN
else
    echo "No options specified. Opening interactive bash terminal in the container..."
    docker exec --env-file $ENV_FILE -it $CONTAINER_ID /entrypoint.sh bash
fi
