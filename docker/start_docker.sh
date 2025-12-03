#!/bin/bash
set -e

# Default values
IMAGE_NAME="walle/ros2:jazzy"
ROS_DOMAIN_ID=62

DOCKER_RUN_FLAGS=()
ENV_FILE="env_file.txt"

# Create the environment file if it doesn't exist
if [ ! -f "$ENV_FILE" ]; then
    echo "Creating environment file: $ENV_FILE"
    echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID" > $ENV_FILE
else
    # Update ROS_DOMAIN_ID in existing file
    if grep -q "^ROS_DOMAIN_ID=" "$ENV_FILE"; then
        sed -i "s/^ROS_DOMAIN_ID=.*/ROS_DOMAIN_ID=$ROS_DOMAIN_ID/" "$ENV_FILE"
    else
        echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID" >> "$ENV_FILE"
    fi
fi

# Actions
# Map each "run" flag variable to the corresponding command to be run in docker
declare -a ACTION_FLAGS=(
    RUN_ROBOT_LAUNCH
    RUN_TELEOP_LAUNCH
    RUN_USB_CAM_NODE
    RUN_VIEW_CAMERA_LAUNCH
    RUN_MOTORS_LAUNCH
    RUN_ROSBRIDGE
    RUN_WEBAPP
)

declare -a ACTION_CMDS=(
    "ros2 launch robot_bringup robot_launch.py"
    "ros2 launch robot_teleop robot_teleop_launch.py"
    "echo 'TODO: implement roslaunch control usb_cam.launch'"
    "echo 'TODO: implement roslaunch control view_camera.launch'"
    "ros2 launch roboclaw_driver roboclaw_launch.py"
    "ros2 launch rosbridge_server rosbridge_websocket_launch.xml"
    "cd /home/walle/web_app/backend && python -m uvicorn main:app --host 0.0.0.0 --port 8000"
)

# Function to show usage
usage() {
    echo "Usage: $0 [ --start (-s) | --teleop (-t) | --usb-cam (-u) \
    | --video-stream (-v) | --motors (-g) | --rosbridge (-B) | --webapp (-w) | --command (-c) <command>]  \
    [ --ros-domain-id (-i) <id> | --copy (-C) <from> <to> | --display (-d) \
    | --build (-b) | --stop (-x) | --restart (-R) | --quiet (-q) | --help (-h) ]"
    echo "This script is used to start and manage a Docker container for WALL-E the wildlife monitoring robot."
    echo "If no IP addresses are specified, the script will attempt to determine them from the hostname. If this fails, try setting the hostname or IP."
    echo "If no action is specified, the script will open an interactive bash terminal in the container."
    echo "Actions (pick ONE or combine):"
    echo "  --start (-s)                Start all processes on the robot"
    echo "  --teleop (-t)               Run joystick control"
    echo "  --usb-cam (-u)              Run usb camera node"
    echo "  --video-stream (-v)         View the video stream"
    echo "  --motors (-m)               Run motor control"
    echo "  --rosbridge (-B)            Run rosbridge server (required for web app)"
    echo "  --webapp (-w)               Run web control panel (port 8000)"
    echo "  --command (-c) <command>    Pass a command to be run in the container"
    echo "Options:"
    echo "  --ros-domain-id (-i) <id>   Set the ROS domain ID (default: $ROS_DOMAIN_ID)"
    echo "  --copy (-C) <from> <to>     Copy files from the container to the host"
    echo "  --display (-d)              Enable display support (forward X11 display)"
    echo "  --build (-b)                Build the Docker container (will stop the running container if any)"
    echo "  --stop (-x)                 Stop the running Docker container"
    echo "  --restart (-R)              Restart the Docker container if it is running"
    echo "  --quiet (-q)                Suppress output"
    echo "  --help (-h)                 Show this help message"
    exit 1
}

RUN_ROBOT_LAUNCH=false
RUN_TELEOP_LAUNCH=false
RUN_USB_CAM_NODE=false
RUN_VIEW_CAMERA_LAUNCH=false
RUN_MOTORS_LAUNCH=false
RUN_ROSBRIDGE=false
RUN_WEBAPP=false
COMMAND_TO_RUN=""

DISPLAY_ENABLED=false
BUILD_CONTAINER=false
STOP_CONTAINER=false
RESTART_CONTAINER=false
QUIET_MODE=false
CLEAN_DOCKER=false

COPY_TO=""
COPY_FROM=""

# Parse options
while [[ "$#" -gt 0 ]]; do
    case "$1" in
        --start|-s) RUN_ROBOT_LAUNCH=true; shift ;;
        --teleop|-t) RUN_TELEOP_LAUNCH=true; shift ;;
        --usb-cam|-u) RUN_USB_CAM_NODE=true; shift ;;
        --video-stream|-v) RUN_VIEW_CAMERA_LAUNCH=true; DISPLAY_ENABLED=true; shift ;;
        --motors|-m) RUN_MOTORS_LAUNCH=true; shift ;;
        --command|-c) COMMAND_TO_RUN="$2"; shift 2 ;;
        --rosbridge|-B) RUN_ROSBRIDGE=true; shift ;;
        --webapp|-w) RUN_WEBAPP=true; shift ;;
        --copy|-C) 
            if [[ -z "$2" || -z "$3" ]]; then
                echo "Error: --copy requires two arguments: <from> <to>"
                usage
            fi
            COPY_FROM="$2"
            COPY_TO="$3"
            shift 3 ;;
        --display|-d) DISPLAY_ENABLED=true; shift ;;
        --build|-b) BUILD_CONTAINER=true; shift ;;
        --stop|-x) STOP_CONTAINER=true; shift ;;
        --restart|-R) RESTART_CONTAINER=true; shift ;;
        --quiet|-q) QUIET_MODE=true; shift ;;
        --clean|-k) CLEAN_DOCKER=true; shift ;;
        --help|-h) usage; shift ;;
        *) usage; shift ;;
    esac
done

# Handle the clean option
if [[ "$CLEAN_DOCKER" = true ]]; then
    echo "Cleaning up Docker system..."
    docker system prune -af --volumes
    echo "Docker cleanup complete."
    exit 0
fi

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

# Add BLINKA_FORCEBOARD environment variable for Adafruit Blinka
DOCKER_RUN_FLAGS+=("--env=BLINKA_FORCEBOARD=RaspberryPi5")

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
    # Change to parent directory to access ros2_ws and ros2_roboclaw_driver
    cd ..
    docker build -t $IMAGE_NAME -f docker/Dockerfile .
    cd docker
fi

# Start the container if it is not already running
if [[ "$RUNNING" = false ]]; then
    echo "Starting the Docker container..."
    docker run -dit --env-file $ENV_FILE "${DOCKER_RUN_FLAGS[@]}" $IMAGE_NAME bash
fi

# Get the container ID for operations
CONTAINER_ID=$(docker ps -q -f ancestor=$IMAGE_NAME) # Get container ID
CONTAINER_ID=$(echo "$CONTAINER_ID" | xargs) # Trim whitespace
echo "Container ID: $CONTAINER_ID"

# Check if the container ID is empty
if [[ -z "$CONTAINER_ID" ]]; then
    echo "Error: Failed to find the Docker container."
    exit 1
fi

# Handle file copying if requested
if [[ -n "$COPY_FROM" && -n "$COPY_TO" ]]; then
    echo "Copying files from container: $COPY_FROM -> $COPY_TO"
    if ! docker cp "$CONTAINER_ID:$COPY_FROM" "$COPY_TO"; then
        echo "Error: Failed to copy files from container."
        exit 1
    fi
    echo "Files copied successfully."
    exit 0
fi

# Run docker commands in detached mode if quiet mode is enabled
if [ "$QUIET_MODE" = true ]; then
    echo "Quiet mode enabled. Suppressing output..."
    # exec 1>/dev/null
    DOCKER_EXEC_FLAGS="-dt"
else
    DOCKER_EXEC_FLAGS="-it"
fi

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
