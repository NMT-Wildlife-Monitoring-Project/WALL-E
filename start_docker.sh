#!/bin/bash
set -e

# Default values
IMAGE_NAME="walle/ros1:noetic"
IP=""
MASTER_IP=""
ROS_MASTER_PORT=11311

DOCKER_RUN_FLAGS=()

DISPLAY_ENABLED=false
COMMAND_TO_RUN=""
ENV_FILE="env_file.txt"
RUN_ROSCORE=false
BUILD_CONTAINER=false
STOP_CONTAINER=false
RESTART_CONTAINER=false
RUN_VIEW_CAMERA_LAUNCH=false
RUN_MAPPING_LAUNCH=false
RUN_VIEW_MAP_LAUNCH=false
RUN_MOTORS_LAUNCH=false
RUN_MOTOR_TELEOP_LAUNCH=false  # NEW flag for motor teleop
QUIET_MODE=false

# Function to show usage
usage() {
    echo "Usage: $0 [--start (-s) | --teleop (-t) | --usb-cam (-u) | --video-stream (-v) |"
    echo "           --mapping (-M) | --view-map (-w) | --motors (-m) | --motor-teleop (-T) | --command (-c) <command> | --roscore (-r) | --build (-b) | --stop (-x) |"
    echo "           --restart (-R)] [--port (-p) <port>] [--ip (-i) <host_ip>] [--master-ip (-m) <master_ip>]"
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
    echo "  --motor-teleop (-T)         Run motor teleop using motor_teleop.launch"  # NEW usage entry
    echo "  --command (-c) <command>    Pass a command to be run in the container"
    echo "  --roscore (-r)              Run roscore"
    echo "Options:"
    echo "  --port (-p) <port>          Specify custom ROS master port (default is 11311)"
    echo "  --ip (-i) <host_ip>         Specify host IP"
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
        --motors|-g) RUN_MOTORS_LAUNCH=true; shift ;;
        --motor-teleop|-T) RUN_MOTOR_TELEOP_LAUNCH=true; shift ;;  # NEW case for motor teleop
        --command|-c) COMMAND_TO_RUN="$2"; shift 2 ;;
        --roscore|-r) RUN_ROSCORE=true; shift ;;
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

# Check if multiple actions are specified
ACTION_COUNT=0
if [ "$RUN_ROBOT_LAUNCH" = true ]; then ACTION_COUNT=$((ACTION_COUNT + 1)); fi
if [ "$RUN_TELEOP_LAUNCH" = true ]; then ACTION_COUNT=$((ACTION_COUNT + 1)); fi
if [ "$RUN_USB_CAM_NODE" = true ]; then ACTION_COUNT=$((ACTION_COUNT + 1)); fi
if [ "$RUN_VIEW_CAMERA_LAUNCH" = true ]; then ACTION_COUNT=$((ACTION_COUNT + 1)); fi
if [ "$RUN_MAPPING_LAUNCH" = true ]; then ACTION_COUNT=$((ACTION_COUNT + 1)); fi
if [ "$RUN_VIEW_MAP_LAUNCH" = true ]; then ACTION_COUNT=$((ACTION_COUNT + 1)); fi
if [ "$RUN_MOTORS_LAUNCH" = true ]; then ACTION_COUNT=$((ACTION_COUNT + 1)); fi
if [ "$RUN_MOTOR_TELEOP_LAUNCH" = true ]; then ACTION_COUNT=$((ACTION_COUNT + 1)); fi  # NEW action count for motor teleop
if [ -n "$COMMAND_TO_RUN" ]; then ACTION_COUNT=$((ACTION_COUNT + 1)); fi
if [ "$RUN_ROSCORE" = true ]; then ACTION_COUNT=$((ACTION_COUNT + 1)); fi

if [ "$ACTION_COUNT" -gt 1 ]; then
    echo "Error: You greedy pig. Multiple actions specified. You get ONE."
    usage
    exit 1
fi

# Execute the specified option
if [ "$RUN_ROBOT_LAUNCH" = true ]; then
    echo "Determining tolerable amounts of sentience..."
    docker exec $DOCKER_EXEC_FLAGS --env-file $ENV_FILE $CONTAINER_ID /entrypoint.sh roslaunch control robot_start.launch
elif [ "$RUN_TELEOP_LAUNCH" = true ]; then
    echo "Running joystick control..."
    docker exec $DOCKER_EXEC_FLAGS --env-file $ENV_FILE $CONTAINER_ID /entrypoint.sh roslaunch control teleop.launch
elif [ "$RUN_USB_CAM_NODE" = true ]; then
    echo "Running usb camera..."
    docker exec $DOCKER_EXEC_FLAGS --env-file $ENV_FILE $CONTAINER_ID /entrypoint.sh roslaunch control usb_cam.launch
elif [ "$RUN_VIEW_CAMERA_LAUNCH" = true ]; then
    echo "Viewing ROS Camera feed..."
    docker exec $DOCKER_EXEC_FLAGS --env-file $ENV_FILE $CONTAINER_ID /entrypoint.sh roslaunch control view_camera.launch
elif [ "$RUN_MAPPING_LAUNCH" = true ]; then
    echo "Running mapping process..."
    docker exec $DOCKER_EXEC_FLAGS --env-file $ENV_FILE $CONTAINER_ID /entrypoint.sh roslaunch control mapping.launch
elif [ "$RUN_VIEW_MAP_LAUNCH" = true ]; then
    echo "Running map view..."
    docker exec $DOCKER_EXEC_FLAGS --env-file $ENV_FILE $CONTAINER_ID /entrypoint.sh roslaunch slamware_ros_sdk view_slamware_ros_sdk_server_node.launch
elif [ "$RUN_MOTORS_LAUNCH" = true ]; then
    echo "Running motor control..."
    docker exec $DOCKER_EXEC_FLAGS --env-file $ENV_FILE $CONTAINER_ID /entrypoint.sh roslaunch motor_control motor_control.launch
elif [ "$RUN_MOTOR_TELEOP_LAUNCH" = true ]; then
    echo "Running motor teleop..."
    docker exec $DOCKER_EXEC_FLAGS --env-file $ENV_FILE $CONTAINER_ID /entrypoint.sh roslaunch control motor_teleop.launch  # NEW execution for motor teleop
elif [ -n "$COMMAND_TO_RUN" ]; then
    echo "Running custom command: $COMMAND_TO_RUN"
    docker exec $DOCKER_EXEC_FLAGS --env-file $ENV_FILE $CONTAINER_ID /entrypoint.sh $COMMAND_TO_RUN
else
    echo "No options specified. Opening interactive bash terminal in the container..."
    docker exec -it --env-file $ENV_FILE $CONTAINER_ID /entrypoint.sh bash
fi
