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
 connect                Connects to an active container
 -i, --image            The name of the image to use to start the container
 -u, --user             Specify the name of the login user. (optional)
 -h, --help             Show this help message and the one from aica-docker
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
   if [ "$( docker container inspect -f '{{.State.Status}}' ${CONTAINER_NAME} 2>/dev/null)" == "running" ]; then
       echo "A container named ${CONTAINER_NAME} is already running. Stopping it."
       docker stop ${CONTAINER_NAME}
   fi

   # Add graphics device access
   FWD_ARGS+=(--device=/dev/dri:/dev/dri)
   
   # Add X11 forwarding for GUI
   FWD_ARGS+=(-e DISPLAY="${DISPLAY}")
   FWD_ARGS+=(-v /tmp/.X11-unix:/tmp/.X11-unix:rw)
   
   # Add environment variables for graphics
   FWD_ARGS+=(-e XDG_RUNTIME_DIR="/tmp/runtime-ros")
   FWD_ARGS+=(-e LIBGL_ALWAYS_SOFTWARE=1)
   
   # Add video group
   FWD_ARGS+=(--group-add video)
   
   # Network setup for ROS and Franka
   FWD_ARGS+=(--net host)
   FWD_ARGS+=(--env ROS_HOSTNAME="$(hostname)")
   FWD_ARGS+=(--env ROS_IP="$ROS_IP")

   # Franka specific settings
   FWD_ARGS+=(--ulimit rtprio=99)
   FWD_ARGS+=(--ulimit rttime=-1)
   FWD_ARGS+=(--device /dev/rtc0:/dev/rtc0)

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
   
   # Create runtime directory with proper permissions
   mkdir -p /tmp/runtime-ros
   chmod 0700 /tmp/runtime-ros
   chown ${USER}:${USER} /tmp/runtime-ros
fi

# Trick aica-docker into making a server on a host network container
if [ "${MODE}" == "server" ]; then
   FWD_ARGS+=("--detach")
   MODE=interactive
fi

if [ "${MODE}" == "" ]; then
   MODE=interactive
fi

# Enable X11 forwarding from docker
xhost +local:docker

# Start docker using aica
aica-docker \
   "${MODE}" \
   "${IMAGE_NAME}" \
   -u "${USERNAME}" \
   -n "${CONTAINER_NAME}" \
   ${GPU_FLAG} \
   "${FWD_ARGS[@]}"

# Ensure runtime directory exists in container
docker exec ${CONTAINER_NAME} bash -c 'mkdir -p /tmp/runtime-ros && chmod 0700 /tmp/runtime-ros && chown ${USER}:${USER} /tmp/runtime-ros' || true