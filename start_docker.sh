#!/bin/bash

# Default values
X11_FORWARD=false
COMMAND=""

# Parse options
while getopts ":xc:" opt; do
  case $opt in
    x)
      X11_FORWARD=true
      ;;
    c)
      COMMAND=$OPTARG
      ;;
    \?)
      echo "Invalid option: -$OPTARG" >&2
      exit 1
      ;;
    :)
      echo "Option -$OPTARG requires an argument." >&2
      exit 1
      ;;
  esac
done

# Run the Docker container with roscore
if [ "$X11_FORWARD" = true ]; then
  # Enable X11 forwarding
  xhost +local:docker
  DOCKER_X11_OPTS="-e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix"
else
  DOCKER_X11_OPTS=""
fi

# Check if the container is already running
if [ "$(docker ps -q -f name=ros1-walle)" ]; then
  echo "Container ros1-walle is already running."
else
  echo "Starting container ros1-walle with roscore..."
  docker run -d --name ros1-walle $DOCKER_X11_OPTS ros1-walle roscore
fi

# Wait for a moment to ensure roscore starts
sleep 3

# Execute command or interactive shell inside the running container
if [ -n "$COMMAND" ]; then
  # Execute the provided command in the container
  docker exec -it ros1-walle bash -c "$COMMAND"
else
  # Start an interactive shell
  docker exec -it ros1-walle bash
fi

# Clean up X11 permissions if X forwarding was enabled
if [ "$X11_FORWARD" = true ]; then
  xhost -local:docker
fi
