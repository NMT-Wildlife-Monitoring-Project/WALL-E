#!/bin/bash
set -e

# Default values
IMAGE_NAME="walle/ros1:noetic"
# MASTER_HOSTNAME="raspberrypi.local"
# MASTER_HOSTNAME="pi"
IP="172.27.32.111"
MASTER_IP="$IP"
ROS_MASTER_PORT=11311

DOCKER_RUN_FLAGS=()

DISPLAY_ENABLED=false
COMMAND_TO_RUN=""
ENV_FILE="env_file.txt"
BUILD_CONTAINER=false
STOP_CONTAINER=false
RESTART_CONTAINER=false
QUIET_MODE=false

# Actions
# Map each “run” flag variable to the corresponding docker command
declare -a ACTION_FLAGS=(
    RUN_ROBOT_LAUNCH
    RUN_TELEOP_LAUNCH
    RUN_USB_CAM_NODE
    RUN_VIEW_CAMERA_LAUNCH
    RUN_MAPPING_LAUNCH
    RUN_VIEW_MAP_LAUNCH
    RUN_MOTORS_LAUNCH
    RUN_ROSBRIDGE
)
declare -a ACTION_CMDS=(
    "roslaunch control robot_start.launch"
    "roslaunch control teleop.launch"
    "roslaunch control usb_cam.launch"
    "roslaunch control view_camera.launch"
    "roslaunch control mapping.launch"
    "roslaunch slamware_ros_sdk view_slamware_ros_sdk_server_node.launch"
    "roslaunch control motor_teleop.launch"
    "roslaunch rosbridge_server rosbridge_websocket.launch"
)

# Function to show usage
usage() {
    echo "Usage: $0 [--start (-s) | --teleop (-t) | --usb-cam (-u) | --video-stream (-v) |"
    echo "           --mapping (-M) | --view-map (-w) | --motors (-m) | --command (-c) <command> | --roscore (-r) | --build (-b) | --stop (-x) |"
    echo "           --restart (-R)] [--port (-p) <port>] [--ip (-i) <local_ip>] [--master-ip (-m) <master_ip>]"
    echo "           [--master-hostname (-n) <master_hostname>] [--display (-d)] [--quiet (-q)] [--help (-h)]"
    echo "This script is used to start and manage a Docker container for WALL-E the wildlife monitoring robot."
    echo "If no IP addresses are specified, the script will attempt to determine them from the hostname. If this fails, try setting the hostname or IP."
    echo "If no action is specified, the script will open an interactive bash terminal in the container."
    echo "Actions (pick ONE):"
    echo "  --start (-s)                Start all processes on the robot"
    echo "  --teleop (-t)               Run joystick control using teleop.launch"
    echo "  --usb-cam (-u)              Run usb camera node using usb_cam.launch"
    echo "  --video-stream (-v)         View the video stream using view_camera.launch"
    echo "  --mapping (-M)              Run mapping process using the slamtec mapper"
    echo "  --view-map (-w)             Run map view using view_slamware_ros_sdk_server_node.launch"
    echo "  --motors (-g)               Run motor control using motor_control.launch"
    echo "  --rosbridge (-B)            Run rosbridge server"
    echo "  --roscore (-r)              Run roscore"
    echo "  --command (-c) <command>    Pass a command to be run in the container"
    echo "Options:"
    echo "  --port (-p) <port>          Specify custom ROS master port (default is 11311)"
    echo "  --ip (-i) <local_ip>         Specify local IP"
    echo "  --master-ip (-m) <master_ip> Specify master IP"
    echo "  --master-hostname (-n) <master_hostname> Specify master hostname (default is raspberrypi.local)"
    echo "  --display (-d)              Enable display support (forward X11 display)"
    echo "  --build (-b)                Build the Docker container (will stop the running container if any)"
    echo "  --stop (-x)                 Stop the running Docker container"
    echo "  --restart (-R)              Restart the Docker container if it is running"
    echo "  --quiet (-q)                Suppress output"
    echo "  --help (-h)                 Show this help message"
    exit 1
}

# Parse options
while [[ "$#" -gt 0 ]]; do
    case "$1" in
        --ip|-i) IP="$2"; shift 2 ;;
        --master-ip|-m) MASTER_IP="$2"; shift 2 ;;
        --master-hostname|-n) MASTER_HOSTNAME="$2"; shift 2 ;;
        --start|-s) RUN_ROBOT_LAUNCH=true; shift ;;
        --teleop|-t) RUN_TELEOP_LAUNCH=true; shift ;;
        --usb-cam|-u) RUN_USB_CAM_NODE=true; shift ;;
        --video-stream|-v) RUN_VIEW_CAMERA_LAUNCH=true; DISPLAY_ENABLED=true; shift ;;
        --mapping|-M) RUN_MAPPING_LAUNCH=true; shift ;;
        --view-map|-w) RUN_VIEW_MAP_LAUNCH=true DISPLAY_ENABLED=true; shift ;;
        --motors|-g) RUN_MOTORS_LAUNCH=true; shift ;;  # NEW case for motors
        --command|-c) COMMAND_TO_RUN="$2"; shift 2 ;;
        --roscore|-r) RUN_ROSCORE=true; shift ;;
        --rosbridge|-B) RUN_ROSBRIDGE=true; shift ;;
        --port|-p) ROS_MASTER_PORT="$2"; shift 2 ;;
        --display|-d) DISPLAY_ENABLED=true; shift ;;
        --build|-b) BUILD_CONTAINER=true; shift ;;
        --stop|-x) STOP_CONTAINER=true; shift ;;
        --restart|-R) RESTART_CONTAINER=true; shift ;;
        --quiet|-q) QUIET_MODE=true; shift ;;
        --help|-h) usage; shift ;;
        *) usage; shift ;;
    esac
done

# Build a list of selected commands
SELECTED_CMDS=()

if [ -n "$COMMAND_TO_RUN" ]; then
    ACTION_FLAGS+=("CUSTOM_COMMAND")
    ACTION_CMDS+=("$COMMAND_TO_RUN")
    SELECTED_CMDS+=("$COMMAND_TO_RUN")
fi

for i in "${!ACTION_FLAGS[@]}"; do
    FLAG_NAME="${ACTION_FLAGS[i]}"
    if [ "${!FLAG_NAME}" = true ]; then
        SELECTED_CMDS+=("${ACTION_CMDS[i]}")
    fi
done

# Print the selected commands
if [ ${#SELECTED_CMDS[@]} -gt 0 ]; then
    echo "Selected commands: ${SELECTED_CMDS[@]}"
else
    echo "No commands selected. Defaulting to interactive bash terminal."
fi

# If multiple commands are selected, automatically enable quiet mode
if [ "${#SELECTED_CMDS[@]}" -gt 1 ]; then
    QUIET_MODE=true
fi

# Check if the container is already running
RUNNING=false
if docker ps -q -f ancestor=$IMAGE_NAME | grep -q .; then
    echo "Docker container is running."
    RUNNING=true
else
    echo "Docker container is not running."
fi

# Stop or restart the container if requested
if [[ "$STOP_CONTAINER" = true || "$RESTART_CONTAINER" = true  || "$BUILD_CONTAINER" = true ]]; then
    if [[ "$RUNNING" = true ]]; then
        echo "Stopping the running Docker container..."
        docker stop $(docker ps -q -f ancestor=$IMAGE_NAME)
        RUNNING=false
    else
        echo "Nothing to stop."
    fi

    if [[ "$STOP_CONTAINER" = true ]]; then
        echo "Exiting..."
        exit 0
    fi
fi

# Function to validate IP address
validate_ip() {
    local ip=$1
    local stat=1

    if [[ $ip =~ ^([0-9]{1,3}\.){3}[0-9]{1,3}$ ]]; then
        OIFS=$IFS
        IFS='.'
        ip=($ip)
        IFS=$OIFS
        [[ ${ip[0]} -le 255 && ${ip[1]} -le 255 && ${ip[2]} -le 255 && ${ip[3]} -le 255 ]]
        stat=$?
    fi
    return $stat
}

# Get the active IP address if not provided
if [[ -z "$IP" ]]; then
    IP=$(ip route get 8.8.8.8 | awk '{print $7; exit}')
    if [[ -z "$IP" ]]; then
        echo "Error: Unable to determine IP address"
        exit 1
    fi
fi

# Validate the IP address
if ! validate_ip "$IP"
then
    echo "Error: Invalid IP address format: $IP"
    exit 1
fi

echo "IP: $IP"

# Determine the master IP if not provided
if [[ -z "$MASTER_IP" && -z "$MASTER_HOSTNAME" ]]; then
    echo "We are the master. You must all obey"
    MASTER_IP=$IP
fi

if [[ -z "$MASTER_IP" || -n "$MASTER_HOSTNAME" ]]; then
    if [[ -n "$MASTER_IP" ]]; then
        OLD_MASTER_IP=$MASTER_IP
    fi

    if [[ "$(hostname)" == "$MASTER_HOSTNAME" ]]; then
        MASTER_IP=$IP
    else
        MASTER_IP=$(ping -c 1 $MASTER_HOSTNAME | grep 'PING' | awk -F'[()]' '{print $2}')
    fi

    if [[ -n "$OLD_MASTER_IP" && "$MASTER_IP" != "$OLD_MASTER_IP" ]]; then
        echo "Warning: IP address discrepancy. Given IP: ($OLD_MASTER_IP), Detected IP: ($MASTER_IP)"
        MASTER_IP=$OLD_MASTER_IP
    fi

    if [[ -z "$MASTER_IP" ]]; then
        echo "Warning: Unable to determine the master IP address from hostname."
        if [[ -n "$OLD_MASTER_IP" ]]; then
            MASTER_IP=$OLD_MASTER_IP
        else
            echo "Error: No master IP address."
            exit 1
        fi
    fi
fi

# Validate the master IP address
if ! validate_ip "$MASTER_IP"; then
    echo "Error: Invalid master IP address format: $MASTER_IP"
    exit 1
fi

echo "Master IP: $MASTER_IP"

# Set ROS_IP and ROS_MASTER_URI
ROS_IP="$IP"
ROS_MASTER_URI="http://$MASTER_IP:$ROS_MASTER_PORT"

# Write environment variables to file
echo "ROS_IP=$ROS_IP" > $ENV_FILE
echo "ROS_MASTER_URI=$ROS_MASTER_URI" >> $ENV_FILE

# No longer part of the Bourgeois 
DOCKER_RUN_FLAGS+=("--privileged")
DOCKER_RUN_FLAGS+=("--net=host")
DOCKER_RUN_FLAGS+=("--cap-add=NET_ADMIN")
DOCKER_RUN_FLAGS+=("--device=/dev/net/tun")
mkdir -p /tmp/shared
DOCKER_RUN_FLAGS+=("--volume=/tmp/shared:/tmp/shared:rw")
DOCKER_RUN_FLAGS+=("--device=/dev/mem")

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

# Start the container if it is not already running
if [[ "$RUNNING" = false ]]; then
    echo "Starting the Docker container..."
    docker run -dit --env-file $ENV_FILE "${DOCKER_RUN_FLAGS[@]}" $IMAGE_NAME bash
fi

CONTAINER_ID=$(docker ps -q -f ancestor=$IMAGE_NAME) # Get container ID
CONTAINER_ID=$(echo "$CONTAINER_ID" | xargs) # Trim whitespace
echo "Container ID: $CONTAINER_ID"

# Check if the container ID is empty
if [[ -z "$CONTAINER_ID" ]]; then
    echo "Error: Failed to start the Docker container."
    exit 1
fi

# Run docker commands in detached mode if quiet mode is enabled
if [ "$QUIET_MODE" = true ]; then
    echo "Quiet mode enabled. Suppressing output..."
    # exec 1>/dev/null
    DOCKER_EXEC_FLAGS="-dt"
else
    DOCKER_EXEC_FLAGS="-it"
fi

# Attempt to run sudo pigpiod in the container
# echo "Starting pigpiod..."
# if ! docker exec $DOCKER_EXEC_FLAGS --env-file $ENV_FILE $CONTAINER_ID sudo pigpiod; then
#     echo "Warning: Failed to start pigpiod. Continuing without it..."
# fi

# Run all selected actions
if [ ${#SELECTED_CMDS[@]} -gt 0 ]; then
    if [ ${#SELECTED_CMDS[@]} -eq 1 ] && [ "$QUIET_MODE" != true ]; then
        # If only one action is specified and not in quiet mode, show output
        cmd="${SELECTED_CMDS[0]}"
        echo "Executing: $cmd"
        docker exec $DOCKER_EXEC_FLAGS --env-file $ENV_FILE $CONTAINER_ID /entrypoint.sh $cmd
    else
        for i in "${!SELECTED_CMDS[@]}"; do
            cmd="${SELECTED_CMDS[i]}"
            echo "Executing: $cmd"
            docker exec $DOCKER_EXEC_FLAGS --env-file $ENV_FILE $CONTAINER_ID /entrypoint.sh $cmd
        done
        echo "Commands are running in the background."
    fi
else
    echo "No actions specified. Opening interactive bash terminal in the container..."
    docker exec -it --env-file $ENV_FILE $CONTAINER_ID /entrypoint.sh bash
fi