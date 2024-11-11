# Handle ros distro
ARG ROS_DISTRO=noetic

FROM ghcr.io/aica-technology/ros-ws:${ROS_DISTRO}

# Avoid interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive
ENV DEBCONF_NONINTERACTIVE_SEEN=true

# Pre-configure keyboard settings
RUN echo "keyboard-configuration keyboard-configuration/layout select Switzerland" | sudo debconf-set-selections && \
    echo "keyboard-configuration keyboard-configuration/variant select Swiss French" | sudo debconf-set-selections && \
    echo "keyboard-configuration keyboard-configuration/model select pc105" | sudo debconf-set-selections

# User provided arguments
ARG HOST_GID=1000
ARG GIT_NAME=""
ARG GIT_EMAIL=""
ARG USE_SIMD=OFF

# Tell docker we want to use bash instead of sh in general
SHELL ["/bin/bash", "-c"]

### Add the user to the current GID of the host to avoid permisson issues in volumes
# AICA uses the same name for user and user group
ENV USER_GROUP=${USER}
USER root
RUN if [ "HOST_GID" != "1000"] ; \
    then groupadd --gid ${HOST_GID} host_group && \
    usermod ${USER} -g ${HOST_GID} && \ 
    usermod ${USER} -a -G ${USER_GROUP}; fi
USER ${USER}

# Setup git identity
RUN git config --global user.name "${GIT_NAME}"
RUN git config --global user.email "${GIT_EMAIL}"

# Setup python version for noetic
RUN sudo apt update
RUN sudo apt install -y python-is-python3

### Add a few tools
RUN sudo apt-get update && sudo apt-get install -y \
    bash-completion \
    silversearcher-ag \
    apt-transport-https \
    less \
    alsa-utils \
    netbase \
    ros-noetic-vrpn-client-ros \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    mesa-utils \
    libgl1-mesa-dri \
    libgl1-mesa-glx \
    mesa-vulkan-drivers \
    xserver-xorg-video-amdgpu \
    && sudo apt-get upgrade -y && sudo apt-get clean

# Install PyTorch and other Python dependencies
RUN pip3 install --no-cache-dir \
    torch \
    torchvision \
    torchaudio

# Handle SIMD option
RUN if [ "${USE_SIMD}" = "ON" ] ; \ 
    then export CMAKE_CXX_FLAGS="-march=native -faligned-new" ; fi
    
# Install Virtual Tactile Pad
WORKDIR /home/${USER}
RUN git clone --depth 1 https://github.com/rayangdn/virtual_tactile_pad.git

RUN cp -R virtual_tactile_pad/ros_ws/src ros_ws/

#Cleaning the workspace
RUN rm -r /home/${USER}/virtual_tactile_pad

### Add environement variables to bashrc
WORKDIR /home/${USER}

# Give bashrc back to user
RUN sudo chown -R ${USER}:${HOST_GID} .bashrc

# Add cmake option to bash rc if needed
RUN if [ "${USE_SIMD}" = "ON" ] ; \
    then echo "export ENABLE_SIMD=ON" >> /home/${USER}/.bashrc ; fi

RUN sudo apt install -y \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    build-essential 

### Build ros workspace
WORKDIR /home/${USER}/ros_ws/
RUN source /opt/ros/noetic/setup.bash && rosdep install --from-paths src --ignore-src -r -y
RUN source /opt/ros/noetic/setup.bash && rosdep install roscpp
RUN source /opt/ros/noetic/setup.bash && catkin_make

WORKDIR /home/${USER}/ros_ws/
# Source the ROS setup file when the container starts
RUN echo "source /opt/ros/noetic/setup.bash && \
                  source devel/setup.bash" >> /home/${USER}/.bashrc

### Final apt clean
RUN sudo apt update && sudo apt upgrade -y && sudo apt clean

# Create and set permissions for runtime directory
RUN sudo mkdir -p /tmp/runtime-ros && \
    sudo chmod 0700 /tmp/runtime-ros && \
    sudo chown ${USER}:${USER} /tmp/runtime-ros

# Set environment variables for graphics
ENV XDG_RUNTIME_DIR="/tmp/runtime-ros"
ENV LIBGL_ALWAYS_SOFTWARE=1

# Keep the DEBIAN_FRONTEND environment variable set to noninteractive
ENV DEBIAN_FRONTEND=noninteractive