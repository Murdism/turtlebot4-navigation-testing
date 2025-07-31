#!/bin/bash
# Run the ROS 2 Jazzy container with optional GPU support
set -e

IMAGE_NAME=ros2_nav2_image
CONTAINER_NAME=ros2_nav2_container
USER_NAME=tester
REPO_DIR=turtlebot4-navigation-testing

# Function to fix ownership after container operations
fix_ownership() {
    echo "Fixing file ownership..."
    sudo chown -R $(id -u):$(id -g) $(pwd) 2>/dev/null || true
}

# Clean build artifacts that may have permission issues
clean_build_artifacts() {
    echo "Cleaning build artifacts..."
    if [ -d "ros2_ws/build" ] || [ -d "ros2_ws/install" ] || [ -d "ros2_ws/log" ]; then
        sudo rm -rf ros2_ws/build ros2_ws/install ros2_ws/log 2>/dev/null || true
    fi
}

# Default GPU usage: auto-detect
USE_GPU="auto"

# Parse command-line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --gpu)
            if [[ "$2" == "true" || "$2" == "false" ]]; then
                USE_GPU="$2"
                shift 2
            else
                echo "Usage: $0 [--gpu true|false] [--clean]"
                exit 1
            fi
            ;;
        --clean)
            clean_build_artifacts
            shift
            ;;
        *)
            shift
            ;;
    esac
done

# Auto-detect GPU if not explicitly set
if [[ "$USE_GPU" == "auto" ]]; then
    if command -v nvidia-smi &>/dev/null && nvidia-smi &>/dev/null; then
        USE_GPU="true"
    else
        USE_GPU="false"
    fi
fi

# Build GPU flag if needed
GPU_FLAG=""
if [[ "$USE_GPU" == "true" ]]; then
    GPU_FLAG="--gpus all --runtime=nvidia"
    echo "🟢 GPU enabled"
else
    echo "⚪️ GPU not used"
fi

# Set permissions and create necessary directories
if ! chmod -R a+rwX $(pwd) 2>/dev/null; then
    echo "Permission issues detected. Cleaning and retrying..."
    clean_build_artifacts
    chmod -R a+rwX $(pwd)
fi

# Create ROS2 workspace directories with proper permissions
mkdir -p ros2_ws/log ros2_ws/build ros2_ws/install
chmod -R 777 ros2_ws/log ros2_ws/build ros2_ws/install

# Permit container to access the host X server for GUI apps (e.g. Gazebo)
xhost +local:docker 2>/dev/null || echo "Warning: Could not set xhost permissions"

# Set up cleanup on exit
trap 'fix_ownership; xhost -local:docker 2>/dev/null || true' EXIT

# Start or exec into the container
if [ "$(docker ps -q -f name=$CONTAINER_NAME)" ]; then
    echo "Container '$CONTAINER_NAME' already running. Connecting to existing container..."
    docker exec -it "$CONTAINER_NAME" bash
else
    echo "Starting new container..."
    # Ensure image exists
    if ! docker image inspect "$IMAGE_NAME" >/dev/null 2>&1; then
        echo "❌ Docker image '$IMAGE_NAME' not found."
        echo "Please run: ./docker/build_docker.sh"
        exit 1
    fi

    # Run the container (remove user mapping to avoid permission issues)
    docker run -it --rm \
        $GPU_FLAG \
        --name "$CONTAINER_NAME" \
        --net=host \
        -e DISPLAY="$DISPLAY" \
        -e QT_X11_NO_MITSHM=1 \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        --mount type=bind,source="$(pwd)",target=/home/tester/$REPO_DIR \
        --user $USER_NAME \
        --device=/dev/dri \
        --privileged \
        "$IMAGE_NAME"
fi