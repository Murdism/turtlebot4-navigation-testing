#!/bin/bash

# Name of the resulting Docker image
IMAGE_NAME=ros2_nav2_image

# Path to this script (to locate the Dockerfile relative to it)
SCRIPT_DIR="$(cd -- "$(dirname "${BASH_SOURCE[0]}")" &>/dev/null && pwd)"
DOCKERFILE_PATH="$SCRIPT_DIR/Dockerfile"

# Verify that the Dockerfile exists
if [ ! -f "$DOCKERFILE_PATH" ]; then
    echo "[✗] Dockerfile not found at: $DOCKERFILE_PATH"
    exit 1
fi

echo "Building ROS 2 Jazzy Docker image: $IMAGE_NAME"
echo "Using Dockerfile: $DOCKERFILE_PATH"

docker build -t "$IMAGE_NAME" -f "$DOCKERFILE_PATH" "$SCRIPT_DIR/.." 

# Check if build was successful
if [ $? -eq 0 ]; then
    echo "[✓] Docker image '$IMAGE_NAME' built successfully."
    echo "[ℹ] You can now run: ./docker/run_docker.sh"
else
    echo "[✗] Failed to build Docker image."
    exit 1
fi