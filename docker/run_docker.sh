#!/bin/bash
# Running container with dynamic resource allocation and persistence options
set -e

IMAGE_NAME=ros2_nav2_image
CONTAINER_NAME=ros2_nav2_container
USER_NAME=tester
REPO_DIR=turtlebot4-navigation-testing

# Default settings
USE_GPU="auto"
CONTAINER_PERSIST="true"  # Default: keep container after exit

# Display usage information
show_usage() {
    cat << EOF
Usage: $0 [OPTIONS]

Container Persistence Options:
  --rm              Remove container after exit (default: keep container)

Resource Options:
  --gpu true|false  Enable/disable GPU support (default: auto-detect)
  --memory SIZE     Memory limit (e.g., 8g, 4g)
  --cpus NUMBER     CPU limit (e.g., 4, 6)
  --shm-size SIZE   Shared memory size (e.g., 2g, 4g)

Examples:
  $0                    # Run with auto-detected resources, keep container
  $0 --rm               # Remove container after exit
  $0 --rm --gpu false   # Remove container, disable GPU
  $0 --memory 8g --cpus 4  # Keep container with custom resources

EOF
}

# Dynamic resource detection
detect_resources() {
    local total_memory_gb=$(free -g | awk '/^Mem:/{print $2}')
    local total_cpus=$(nproc)
    
    echo "System Resources: ${total_memory_gb}GB RAM, ${total_cpus} CPUs"
    
    # Calculate optimal allocations (reserve resources for host system)
    if [ "$total_memory_gb" -gt 16 ]; then
        MEMORY_LIMIT="12g"
        SHM_SIZE="4g"
    elif [ "$total_memory_gb" -gt 8 ]; then
        MEMORY_LIMIT="6g"
        SHM_SIZE="2g"
    else
        MEMORY_LIMIT="4g"
        SHM_SIZE="1g"
    fi
    
    if [ "$total_cpus" -gt 8 ]; then
        CPU_LIMIT="6"
    elif [ "$total_cpus" -gt 4 ]; then
        CPU_LIMIT="4"
    else
        CPU_LIMIT="2"
    fi
    
    echo "Allocated: ${CPU_LIMIT} CPUs, ${MEMORY_LIMIT} RAM, ${SHM_SIZE} SHM"
}

# Parse command-line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --rm)
            CONTAINER_PERSIST="false"
            shift
            ;;
        --gpu)
            if [[ "$2" == "true" || "$2" == "false" ]]; then
                USE_GPU="$2"
                shift 2
            else
                echo "Error: --gpu requires 'true' or 'false'"
                show_usage
                exit 1
            fi
            ;;
        --memory)
            MEMORY_LIMIT="$2"
            shift 2
            ;;
        --cpus)
            CPU_LIMIT="$2"
            shift 2
            ;;
        --shm-size)
            SHM_SIZE="$2"
            shift 2
            ;;
        --help|-h)
            show_usage
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            show_usage
            exit 1
            ;;
    esac
done

# Auto-detect resources if not manually specified
if [ -z "$MEMORY_LIMIT" ] || [ -z "$CPU_LIMIT" ]; then
    detect_resources
fi

# Auto-detect GPU if not explicitly set
if [[ "$USE_GPU" == "auto" ]]; then
    if command -v nvidia-smi &>/dev/null && nvidia-smi &>/dev/null; then
        USE_GPU="true"
        echo "GPU enabled"
    else
        USE_GPU="false"
        echo "GPU not available"
    fi
fi

# Build GPU flags
GPU_FLAGS=""
if [[ "$USE_GPU" == "true" ]]; then
    GPU_FLAGS="--gpus all --runtime=nvidia"
fi

# Build persistence flags
PERSISTENCE_FLAGS=""
if [[ "$CONTAINER_PERSIST" == "false" ]]; then
    PERSISTENCE_FLAGS="--rm"
fi

# Enable X11 forwarding for GUI applications
xhost +local:docker 2>/dev/null || echo "Warning: Could not set X11 permissions"

# Check if container already exists (stopped or running)
container_exists() {
    docker ps -a -q -f name="^${CONTAINER_NAME}$" | grep -q .
}

container_running() {
    docker ps -q -f name="^${CONTAINER_NAME}$" | grep -q .
}

# Handle existing containers
if container_running; then
    echo "Container '$CONTAINER_NAME' is already running. Connecting..."
    docker exec -it "$CONTAINER_NAME" bash
    exit 0
elif container_exists; then
    echo "Found stopped container '$CONTAINER_NAME'."
    read -p "Do you want to (r)estart it, (d)elete and create new, or (c)ancel? [r/d/c]: " choice
    case $choice in
        r|R)
            echo "Restarting existing container..."
            docker start "$CONTAINER_NAME"
            docker exec -it "$CONTAINER_NAME" bash
            exit 0
            ;;
        d|D)
            echo "Removing existing container..."
            docker rm "$CONTAINER_NAME"
            ;;
        c|C|*)
            echo "Operation cancelled."
            exit 0
            ;;
    esac
fi

# Ensure image exists
if ! docker image inspect "$IMAGE_NAME" >/dev/null 2>&1; then
    echo "Error: Docker image '$IMAGE_NAME' not found."
    echo "Please run: ./docker/build_docker.sh"
    exit 1
fi

# Display container persistence info
if [[ "$CONTAINER_PERSIST" == "false" ]]; then
    echo "Starting temporary container (will be removed after exit)"
else
    echo "Starting persistent container (use 'docker stop $CONTAINER_NAME' to stop, 'docker start $CONTAINER_NAME' to restart)"
fi

echo "Resource allocation: ${CPU_LIMIT} CPUs, ${MEMORY_LIMIT} RAM, ${SHM_SIZE} SHM"

# Run container with dynamic resources and persistence options
docker run -it \
    $PERSISTENCE_FLAGS \
    $GPU_FLAGS \
    --name "$CONTAINER_NAME" \
    --network host \
    --memory="$MEMORY_LIMIT" \
    --cpus="$CPU_LIMIT" \
    --shm-size="$SHM_SIZE" \
    --ulimit nofile=65536:65536 \
    --ulimit nproc=4096:4096 \
    --env DISPLAY="$DISPLAY" \
    --env QT_X11_NO_MITSHM=1 \
    --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
    --volume "$(pwd)":/home/$USER_NAME/$REPO_DIR:rw \
    --user "$USER_NAME" \
    --device=/dev/dri \
    "$IMAGE_NAME"