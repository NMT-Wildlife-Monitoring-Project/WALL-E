#!/bin/bash
set -e

# ============================================================================
# WALL-E Docker Container Manager
# ============================================================================
# Manages Docker container for WALL-E robot across multiple platforms
# (Raspberry Pi, Nvidia Jetson, x86/x64 computers)
# ============================================================================

# ============================================================================
# CONFIGURATION
# ============================================================================
IMAGE_NAME="walle/ros2:jazzy"
ROS_DOMAIN_ID=62
ENV_FILE="env_file.txt"

# ============================================================================
# GLOBAL STATE
# ============================================================================
# Action flags - Core Operations
RUN_ROBOT_LAUNCH=false
RUN_NAVIGATE=false
RUN_TELEOP_LAUNCH=false

# Action flags - Sensors
RUN_LIDAR=false
RUN_CAMERA=false
RUN_IMU=false
RUN_GPS=false

# Action flags - Debugging & Visualization
RUN_RVIZ=false
RUN_LIDAR_VIEW=false
RUN_ROSBRIDGE=false
RUN_SCAN_MATCHER=false
RUN_WAYPOINT_SERVER=false

# Action flags - Advanced & Specialized
RUN_EKF=false
RUN_MOTORS_LAUNCH=false

# Action flags - Web Interface
RUN_WEBAPP=false
COMMAND_TO_RUN=""

# Control flags
BUILD_CONTAINER=false
STOP_CONTAINER=false
RESTART_CONTAINER=false
CLEAN_DOCKER=false
DISPLAY_ENABLED=false
QUIET_MODE=false

# File operations
COPY_FROM=""
COPY_TO=""

# Runtime state
RUNNING=false
SELECTED_CMDS=()
DOCKER_RUN_FLAGS=()

# ============================================================================
# ACTION MAPPINGS (Flags → Commands)
# ============================================================================
# Core Operations
declare -a CORE_OPS_FLAGS=(
    RUN_ROBOT_LAUNCH
    RUN_NAVIGATE
    RUN_TELEOP_LAUNCH
)

declare -a CORE_OPS_CMDS=(
    "ros2 launch robot_bringup robot_launch.py"
    "ros2 launch robot_navigation gps_waypoint_follower.launch.py"
    "ros2 launch robot_teleop robot_teleop_launch.py"
)

# Sensors (often used for testing)
declare -a SENSORS_FLAGS=(
    RUN_LIDAR
    RUN_CAMERA
    RUN_IMU
    RUN_GPS
)

declare -a SENSORS_CMDS=(
    "ros2 launch sllidar_ros2 sllidar_s3_launch.py"
    "ros2 launch robot_bringup usb_cam_launch.py"
    "ros2 launch bno085_driver bno085_launch.py"
    "ros2 launch robot_bringup gps_launch.py"
)

# Debugging & Visualization
declare -a DEBUG_VIZ_FLAGS=(
    RUN_RVIZ
    RUN_LIDAR_VIEW
    RUN_ROSBRIDGE
    RUN_SCAN_MATCHER
    RUN_WAYPOINT_SERVER
)

declare -a DEBUG_VIZ_CMDS=(
    "rviz2 -d /home/walle/.rviz2/default.rviz"
    "ros2 launch sllidar_ros2 view_sllidar_s3_launch.py"
    "ros2 launch rosbridge_server rosbridge_websocket_launch.xml"
    "ros2 launch scan_matcher scan_matcher_launch.py"
    "ros2 launch waypoint_server launch_waypoint_system.launch.py"
)

# Advanced & Specialized
declare -a ADVANCED_FLAGS=(
    RUN_EKF
    RUN_MOTORS_LAUNCH
)

declare -a ADVANCED_CMDS=(
    "ros2 launch robot_navigation dual_ekf_navsat.launch.py"
    "ros2 launch roboclaw_driver roboclaw_launch.py"
)

# Web Interface
declare -a WEB_FLAGS=(
    RUN_WEBAPP
)

declare -a WEB_CMDS=(
    "cd /home/walle/web_app/backend && python -m uvicorn main:app --host 0.0.0.0 --port 8000"
)

# Combine all action arrays
declare -a ACTION_FLAGS=(
    "${CORE_OPS_FLAGS[@]}"
    "${SENSORS_FLAGS[@]}"
    "${DEBUG_VIZ_FLAGS[@]}"
    "${ADVANCED_FLAGS[@]}"
    "${WEB_FLAGS[@]}"
)

declare -a ACTION_CMDS=(
    "${CORE_OPS_CMDS[@]}"
    "${SENSORS_CMDS[@]}"
    "${DEBUG_VIZ_CMDS[@]}"
    "${ADVANCED_CMDS[@]}"
    "${WEB_CMDS[@]}"
)

# ============================================================================
# HELPER FUNCTIONS
# ============================================================================

# Print usage information
usage() {
    cat << 'USAGE'
Usage: ./start_docker.sh [OPTIONS]

CORE OPERATIONS (essential robot functionality)
  -r, --robot              Start all robot processes (robot bringup)
  -n, --navigate           Start GPS waypoint navigation
  -t, --teleop             Run joystick/teleop control

SENSORS (often used for testing individual sensors)
  -l, --lidar              Start LiDAR (S-Lidar S3)
  -c, --camera             Start USB camera
  --imu                    Start IMU sensor (BNO085)
  --gps                    Start GPS/NMEA driver

DEBUGGING & VISUALIZATION (development and diagnostics)
  -R, --rviz               Start RViz 2 with default config (requires -d)
  -v, --lidar-view         Visualize LiDAR in RViz (requires -d)
  -B, --rosbridge          Run rosbridge server (required for web app)
  --scan-matcher           Start ICP scan matching (odometry refinement)
  --waypoint-server        Start waypoint management server

ADVANCED & SPECIALIZED (advanced navigation and system tuning)
  --ekf                    Start EKF localization (dual filter: odom + map)
  -m, --motors             Run motor control (roboclaw driver)

WEB INTERFACE
  -w, --webapp             Run web control panel (port 8000)

CUSTOM
  --command <cmd>          Run custom command in container

CONTAINER MANAGEMENT
  -b, --build              Build Docker image (stops running container if any)
  -x, --stop               Stop the running container
  -R, --restart            Restart the container if running
  -k, --clean              Clean up Docker system (prune all unused images/volumes)

OPTIONS
  -i, --ros-domain-id <id> Set ROS domain ID (default: 62)
  -C, --copy <from> <to>   Copy files from container to host
  -d, --display            Enable X11 display forwarding (for RViz, etc)
  -q, --quiet              Run commands in background (detached mode)
  -h, --help               Show this help message

EXAMPLES
  ./start_docker.sh -b                         # Build image
  ./start_docker.sh -r                         # Start robot bringup
  ./start_docker.sh -r -n                      # Robot + navigation
  ./start_docker.sh -l -t                      # LiDAR + teleop control
  ./start_docker.sh -R -d                      # Start RViz with display
  ./start_docker.sh -r -R -d                   # Robot + RViz visualization
  ./start_docker.sh -r --ekf                   # Robot with EKF localization
  ./start_docker.sh -w -B                      # Web app + rosbridge
  ./start_docker.sh -c "ros2 topic list"       # Run custom command
  ./start_docker.sh -d                         # Interactive bash with display

DEFAULT BEHAVIOR (no actions specified)
  Opens interactive bash terminal in the container
USAGE
    exit 1
}

# Auto-detect platform-specific hardware settings
detect_hardware() {
    local board_name=""

    # Detect Jetson
    if grep -qi "tegra" /proc/device-tree/model 2>/dev/null || \
       grep -qi "jetson" /proc/device-tree/model 2>/dev/null || \
       [ -f "/sys/module/tegra_fuse/parameters/tegra_chip_id" ]; then
        board_name="Jetson"
        export BLINKA_FORCEBOARD="JETSON_TX2"
    # Detect Raspberry Pi
    elif grep -qi "raspberry pi" /proc/device-tree/model 2>/dev/null; then
        board_name="Raspberry Pi"
        export BLINKA_FORCEBOARD="RASPBERRY_PI_5"
    # Default x86
    else
        board_name="x86/x64"
        export BLINKA_FORCEBOARD=""
    fi

    echo "Detected hardware: $board_name"
    if [ -n "$BLINKA_FORCEBOARD" ]; then
        echo "Setting BLINKA_FORCEBOARD=$BLINKA_FORCEBOARD"
    fi
}

# Setup or update environment file
setup_environment_file() {
    if [ ! -f "$ENV_FILE" ]; then
        echo "Creating environment file: $ENV_FILE"
        echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID" > "$ENV_FILE"
    else
        # Update ROS_DOMAIN_ID in existing file
        if grep -q "^ROS_DOMAIN_ID=" "$ENV_FILE"; then
            sed -i "s/^ROS_DOMAIN_ID=.*/ROS_DOMAIN_ID=$ROS_DOMAIN_ID/" "$ENV_FILE"
        else
            echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID" >> "$ENV_FILE"
        fi
    fi
}

# Add display support to environment and docker flags
enable_display_support() {
    if [ -z "$DISPLAY" ]; then
        echo "Warning: DISPLAY not set. GUI applications may not work."
        echo "         Ensure you have X11 forwarding enabled or use: export DISPLAY=:0"
        return
    fi

    if [ ! -e /tmp/.X11-unix ]; then
        echo "Warning: X11 socket not found at /tmp/.X11-unix"
        echo "         GUI applications may not work without X11."
        return
    fi

    echo "DISPLAY=$DISPLAY" >> "$ENV_FILE"
    DOCKER_RUN_FLAGS+=("--volume=/tmp/.X11-unix:/tmp/.X11-unix:rw")
    xhost +local:docker 2>/dev/null || true
    echo "Display support enabled"
}

# Check if Docker container is running
check_container_status() {
    if docker ps -q -f ancestor="$IMAGE_NAME" | grep -q .; then
        RUNNING=true
        echo "Docker container is running."
    else
        RUNNING=false
        echo "Docker container is not running."
    fi
}

# Stop the running container
stop_container() {
    if [ "$RUNNING" = true ]; then
        echo "Stopping the running Docker container..."
        docker stop "$(docker ps -q -f ancestor="$IMAGE_NAME")"
        RUNNING=false
    else
        echo "Nothing to stop."
    fi
}

# Build Docker image
build_image() {
    echo "Building the Docker container..."
    # Change to parent directory to access ros2_ws and other build context
    cd ..
    docker build -t "$IMAGE_NAME" -f docker/Dockerfile .
    cd docker
    echo "Build complete."
}

# Start Docker container
start_container() {
    if [ "$RUNNING" = false ]; then
        echo "Starting the Docker container..."
        docker run -dit --env-file "$ENV_FILE" "${DOCKER_RUN_FLAGS[@]}" "$IMAGE_NAME" bash
    fi
}

# Get container ID and validate it exists
get_container_id() {
    local container_id
    container_id=$(docker ps -q -f ancestor="$IMAGE_NAME" | xargs)

    if [ -z "$container_id" ]; then
        echo "Error: Failed to find the Docker container."
        exit 1
    fi

    echo "$container_id"
}

# Copy files from container to host
copy_files_from_container() {
    local container_id
    container_id=$(get_container_id)

    echo "Copying files from container: $COPY_FROM -> $COPY_TO"
    if ! docker cp "$container_id:$COPY_FROM" "$COPY_TO"; then
        echo "Error: Failed to copy files from container."
        exit 1
    fi
    echo "Files copied successfully."
}

# Build list of selected commands to run
build_command_list() {
    # Add custom command if specified
    if [ -n "$COMMAND_TO_RUN" ]; then
        SELECTED_CMDS+=("$COMMAND_TO_RUN")
    fi

    # Add action-based commands
    for i in "${!ACTION_FLAGS[@]}"; do
        local flag_name="${ACTION_FLAGS[i]}"
        if [ "${!flag_name}" = true ]; then
            SELECTED_CMDS+=("${ACTION_CMDS[i]}")
        fi
    done

    # Auto-enable display for visualization commands that require it
    if [ "$RUN_RVIZ" = true ] || [ "$RUN_LIDAR_VIEW" = true ]; then
        DISPLAY_ENABLED=true
    fi

    # Print summary
    if [ ${#SELECTED_CMDS[@]} -gt 0 ]; then
        echo "Selected commands: ${SELECTED_CMDS[*]}"
    else
        echo "No commands selected. Defaulting to interactive bash terminal."
    fi

    # Auto-enable quiet mode for multiple commands
    if [ ${#SELECTED_CMDS[@]} -gt 1 ]; then
        QUIET_MODE=true
    fi
}

# Execute commands in container
execute_commands() {
    local container_id
    container_id=$(get_container_id)
    echo "Container ID: $container_id"

    if [ ${#SELECTED_CMDS[@]} -eq 0 ]; then
        # Interactive mode: no commands selected
        echo "Opening interactive bash terminal in the container..."
        docker exec -it --env-file "$ENV_FILE" "$container_id" /entrypoint.sh bash
    elif [ ${#SELECTED_CMDS[@]} -eq 1 ] && [ "$QUIET_MODE" != true ]; then
        # Single command, interactive output
        local cmd="${SELECTED_CMDS[0]}"
        echo "Executing: $cmd"
        docker exec -it --env-file "$ENV_FILE" "$container_id" /entrypoint.sh "$cmd"
    else
        # Multiple commands or quiet mode: detached execution
        if [ "$QUIET_MODE" = true ]; then
            echo "Quiet mode enabled. Running commands in background..."
        fi

        for cmd in "${SELECTED_CMDS[@]}"; do
            echo "Executing: $cmd"
            docker exec -dt --env-file "$ENV_FILE" "$container_id" /entrypoint.sh "$cmd"
        done

        echo "Commands are running in the background."
    fi
}

# ============================================================================
# MAIN SCRIPT LOGIC
# ============================================================================

# Parse command-line arguments
while [[ "$#" -gt 0 ]]; do
    case "$1" in
        # Core Operations
        -r|--robot) RUN_ROBOT_LAUNCH=true; shift ;;
        -n|--navigate) RUN_NAVIGATE=true; shift ;;
        -t|--teleop) RUN_TELEOP_LAUNCH=true; shift ;;

        # Sensors
        -l|--lidar) RUN_LIDAR=true; shift ;;
        -c|--camera) RUN_CAMERA=true; shift ;;
        --imu) RUN_IMU=true; shift ;;
        --gps) RUN_GPS=true; shift ;;

        # Debugging & Visualization
        -R|--rviz) RUN_RVIZ=true; shift ;;
        -v|--lidar-view) RUN_LIDAR_VIEW=true; shift ;;
        -B|--rosbridge) RUN_ROSBRIDGE=true; shift ;;
        --scan-matcher) RUN_SCAN_MATCHER=true; shift ;;
        --waypoint-server) RUN_WAYPOINT_SERVER=true; shift ;;

        # Advanced & Specialized
        --ekf) RUN_EKF=true; shift ;;
        -m|--motors) RUN_MOTORS_LAUNCH=true; shift ;;

        # Web Interface
        -w|--webapp) RUN_WEBAPP=true; shift ;;

        # Custom
        --command) COMMAND_TO_RUN="$2"; shift 2 ;;

        # Container management
        -b|--build) BUILD_CONTAINER=true; shift ;;
        -x|--stop) STOP_CONTAINER=true; shift ;;
        --restart) RESTART_CONTAINER=true; shift ;;
        -k|--clean) CLEAN_DOCKER=true; shift ;;

        # Options
        -i|--ros-domain-id) ROS_DOMAIN_ID="$2"; shift 2 ;;
        -C|--copy)
            if [[ -z "$2" || -z "$3" ]]; then
                echo "Error: --copy requires two arguments: <from> <to>"
                usage
            fi
            COPY_FROM="$2"
            COPY_TO="$3"
            shift 3
            ;;
        -d|--display) DISPLAY_ENABLED=true; shift ;;
        -q|--quiet) QUIET_MODE=true; shift ;;
        -h|--help) usage ;;
        *) usage ;;
    esac
done

# Handle early exits
if [ "$CLEAN_DOCKER" = true ]; then
    echo "Cleaning up Docker system..."
    docker system prune -af --volumes
    echo "Docker cleanup complete."
    exit 0
fi

# Setup and validation
setup_environment_file
detect_hardware

# Container management workflow
check_container_status

if [ "$STOP_CONTAINER" = true ] || [ "$RESTART_CONTAINER" = true ] || [ "$BUILD_CONTAINER" = true ]; then
    stop_container

    if [ "$STOP_CONTAINER" = true ]; then
        echo "Container stopped. Exiting."
        exit 0
    fi
fi

# Configure Docker runtime flags
# ============================================================================
# Network and device access (required for hardware)
DOCKER_RUN_FLAGS+=("--privileged")
DOCKER_RUN_FLAGS+=("--net=host")
DOCKER_RUN_FLAGS+=("--cap-add=NET_ADMIN")
DOCKER_RUN_FLAGS+=("--device=/dev/net/tun")
DOCKER_RUN_FLAGS+=("--device=/dev/mem")
DOCKER_RUN_FLAGS+=("--volume=/dev:/dev:rw")

# Shared memory
mkdir -p /tmp/shared
DOCKER_RUN_FLAGS+=("--volume=/tmp/shared:/tmp/shared:rw")

# Hardware-specific environment
if [ -n "$BLINKA_FORCEBOARD" ]; then
    DOCKER_RUN_FLAGS+=("--env=BLINKA_FORCEBOARD=$BLINKA_FORCEBOARD")
fi

# USB camera support (if requested)
if [ "$RUN_CAMERA" = true ]; then
    if [ -e /dev/video0 ]; then
        DOCKER_RUN_FLAGS+=("--device=/dev/video0")
    else
        echo "Error: USB camera requested but /dev/video0 does not exist."
        exit 1
    fi
fi

# Display support (if requested)
if [ "$DISPLAY_ENABLED" = true ]; then
    enable_display_support
fi

# Build image if requested
if [ "$BUILD_CONTAINER" = true ]; then
    build_image
    check_container_status  # Refresh status after build
fi

# Start container if not already running
start_container

# File operations (if requested)
if [ -n "$COPY_FROM" ] && [ -n "$COPY_TO" ]; then
    copy_files_from_container
    exit 0
fi

# Build and execute command list
build_command_list
execute_commands
