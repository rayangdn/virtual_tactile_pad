#!/bin/bash

IMAGE_NAME="epfl-lasa/virtual_tactile_pad"
CONTAINER_NAME="${IMAGE_NAME//[\/.]/-}"
USERNAME="ros"
MODE=()
USE_NVIDIA_TOOLKIT=false

# Help
HELP_MESSAGE="Usage: ./start_dockers.sh [interactive | server | connect] [-i, --image] [-u, --user]
Build the '${IMAGE_NAME}' image.
Options:
 interactive            Spin the image in the console
 server                 Spin the image as an ssh server
 connect               Connects to an active container
 -i, --image           The name of the image to use to start the container
 -u, --user            Specify the name of the login user. (optional)
 -h, --help            Show this help message and the one from aica-docker
 Additional arguments are passed to the aica-docker command.
"

# Argument parsing
RUN_FLAGS=()
FWS_FLAGS=()
SHOW_HELP=false

while [ "$#" -gt 0 ]; do
    case "$1" in
    -i | --image)
        IMAGE_NAME=$2
        shift 2
        ;;
    -u | --user)
        USERNAME=$2
        shift 2
        ;;
    -m | --mode)
        MODE=$2
        shift 2
        ;;
    -h | --help)
        SHOW_HELP=true
        shift 1
        ;;
    *)
        if [ -z "${MODE}" ]; then
            MODE=$1
        else
            FWD_ARGS+=("$1")
        fi
        shift 1
        ;;
    esac
done

# Help verbose
if $SHOW_HELP; then
    echo $HELP_MESSAGE
    aica-docker $MODE -h
    exit 1
fi

# Handle interactive/server specific arguments
if [ "${MODE}" != "connect" ]; then
    # Check if a container with this name is already running
    if [ "$(docker container inspect -f '{{.State.Status}}' ${CONTAINER_NAME} 2>/dev/null)" == "running" ]; then
        echo "A container named ${CONTAINER_NAME} is already running. Stopping it."
        docker stop ${CONTAINER_NAME}
    fi

    # Allow X server connections from the container
    xhost +local:docker

    # Graphics and display related arguments
    FWD_ARGS+=(--device=/dev/dri:/dev/dri)
    FWD_ARGS+=(-e DISPLAY=$DISPLAY)
    FWD_ARGS+=(-e QT_X11_NO_MITSHM=1)
    FWD_ARGS+=(-v /tmp/.X11-unix:/tmp/.X11-unix:rw)

    # Set XDG_RUNTIME_DIR
    FWD_ARGS+=(-e XDG_RUNTIME_DIR=/tmp/runtime-ros)
    
    # network for ros
    FWD_ARGS+=(--net=host) # Share host IP
    FWD_ARGS+=(--env ROS_HOSTNAME="$(hostname)")

    # Handle GPU usage
    [[ ${USE_NVIDIA_TOOLKIT} = true ]] && GPU_FLAG="--gpus all" || GPU_FLAG=""

    # Privileged mode
    FWD_ARGS+=("--privileged")

    # Add volume src
    docker volume rm virtual_tactile_pad
    docker volume create --driver local \
        --opt type="none" \
        --opt device="${PWD}/../ros_ws/src/" \
        --opt o="bind" \
        "virtual_tactile_pad"
    FWD_ARGS+=(--volume="virtual_tactile_pad:/home/ros/ros_ws/src:rw")
fi

# Trick aica-docker into making a server on a host network container
if [ "${MODE}" == "server" ]; then
    FWD_ARGS+=("--detach")
    MODE=interactive
fi

if [ "${MODE}" == "" ]; then
    MODE=interactive
fi

# Create runtime directory if it doesn't exist
mkdir -p /tmp/runtime-ros
chmod 700 /tmp/runtime-ros

# Start docker using aica
aica-docker \
    "${MODE}" \
    "${IMAGE_NAME}" \
    -u "${USERNAME}" \
    -n "${CONTAINER_NAME}" \
    ${GPU_FLAG} \
    "${FWD_ARGS[@]}"