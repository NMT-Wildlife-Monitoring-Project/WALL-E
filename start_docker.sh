#!/bin/bash

# Define variables
CONTAINER_NAME="ros_noetic_container"
IMAGE_NAME="ros_noetic_image"

# Function to show usage
usage() {
    echo "Usage: $0 [-r | -t | -c <command>] [-h]"
    echo "  -r          Run robot_start.launch from package control"
    echo "  -t          Run teleop.launch from package control"
    echo "  -c <command> Pass a command to be run in the container"
    echo "  -h          Show this help message"
    echo "If no options are given, an interactive bash terminal will be opened in the container."
    exit 1
}

# Parse options
while getopts "rtc:h" opt; do
    case "$opt" in
        r) RUN_ROBOT_LAUNCH=true ;;
        t) RUN_TELEOP_LAUNCH=true ;;
        c) COMMAND_TO_RUN="$OPTARG" ;;
        h) usage ;;
        *) usage ;;
    esac
done

# Build the container image
echo "Building Docker image..."
docker build -t $IMAGE_NAME .

# Start the container if not already running
if [ ! "$(docker ps -q -f name=$CONTAINER_NAME)" ]; then
    if [ "$(docker ps -aq -f status=exited -f name=$CONTAINER_NAME)" ]; then
        # Remove the stopped container
        docker rm $CONTAINER_NAME
    fi
    echo "Starting Docker container with roscore..."
    docker run -dit --name $CONTAINER_NAME --privileged --entrypoint bash $IMAGE_NAME -c "source /opt/ros/noetic/setup.bash && source ~/catkin_ws/devel/setup.bash && roscore"
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
