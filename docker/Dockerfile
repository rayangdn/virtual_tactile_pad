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

# Add the user to the current GID of the host to avoid permission issues in volumes
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

# Install system dependencies
RUN sudo apt-get update && sudo apt-get install -y \
    python-is-python3 \
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
    python3-matplotlib \
    python3-numpy \
    python3-pandas \
    python3-tk \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    build-essential \
    cmake \
    git \
    libpoco-dev \
    libeigen3-dev \
    && sudo apt-get upgrade -y && sudo apt-get clean

# Install Python dependencies
RUN pip3 install --no-cache-dir \
    torch \
    torchvision \
    torchaudio \
    matplotlib \
    numpy \
    pandas \
    scipy \
    seaborn \
    scikit-learn \
    pathlib

# Handle SIMD option
RUN if [ "${USE_SIMD}" = "ON" ] ; \ 
    then export CMAKE_CXX_FLAGS="-march=native -faligned-new" ; fi

# Install libfranka
WORKDIR /home/${USER}/ros_ws/src
RUN git clone --recursive https://github.com/frankaemika/libfranka && \
    cd libfranka && \
    git checkout 0.8.0 && \
    git submodule update && \
    mkdir build && \
    cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF .. && \
    cmake --build . && \
    sudo make install -j && \
    sudo ldconfig

# Install franka_ros
WORKDIR /home/${USER}/ros_ws/src
RUN git clone --recursive https://github.com/frankaemika/franka_ros --branch noetic-devel && \
    cd franka_ros && \
    git checkout 0.8.0 && \
    git submodule update

# Install Virtual Tactile Pad
WORKDIR /home/${USER}
RUN git clone --depth 1 https://github.com/rayangdn/virtual_tactile_pad.git && \
    cp -R virtual_tactile_pad/ros_ws/src ros_ws/ && \
    rm -r /home/${USER}/virtual_tactile_pad

# Setup ROS workspace
WORKDIR /home/${USER}/ros_ws
RUN rosdep install --from-paths src --ignore-src --rosdistro noetic -y --skip-keys libfranka && \
    source /opt/ros/$ROS_DISTRO/setup.bash && \
    catkin_make -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/home/${USER}/ros_ws/src/libfranka/build

# Configure environment
WORKDIR /home/${USER}
RUN sudo chown -R ${USER}:${HOST_GID} .bashrc

# Add cmake option to bashrc if needed
RUN if [ "${USE_SIMD}" = "ON" ] ; \
    then echo "export ENABLE_SIMD=ON" >> /home/${USER}/.bashrc ; fi

# Source ROS setup files
RUN echo "source /opt/ros/noetic/setup.bash && \
    source /home/${USER}/ros_ws/devel/setup.bash" >> /home/${USER}/.bashrc

# Create and set permissions for runtime directory
RUN sudo mkdir -p /tmp/runtime-ros && \
    sudo chmod 0700 /tmp/runtime-ros && \
    sudo chown ${USER}:${USER} /tmp/runtime-ros

# Set environment variables for graphics
ENV XDG_RUNTIME_DIR="/tmp/runtime-ros"
ENV LIBGL_ALWAYS_SOFTWARE=1
ENV DEBIAN_FRONTEND=noninteractive

WORKDIR /home/${USER}/ros_ws