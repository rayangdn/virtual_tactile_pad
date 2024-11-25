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

# Setup git identity as user
USER ${USER}
RUN git config --global user.name "${GIT_NAME}"
RUN git config --global user.email "${GIT_EMAIL}"

# Install robotpkg and Pinocchio dependencies
USER root
RUN apt-get update && apt-get install -qqy lsb-release curl && \
    mkdir -p /etc/apt/keyrings && \
    curl http://robotpkg.openrobots.org/packages/debian/robotpkg.asc | tee /etc/apt/keyrings/robotpkg.asc && \
    echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" \
    | tee /etc/apt/sources.list.d/robotpkg.list && \
    apt-get update

# Install system dependencies
RUN apt-get install -y \
    python-is-python3 \
    bash-completion \
    silversearcher-ag \
    apt-transport-https \
    less \
    alsa-utils \
    netbase \
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
    python3-sklearn \
    build-essential \
    cmake \
    git \
    libpoco-dev \
    libeigen3-dev \
    python3-pip \
    && apt-get upgrade -y && apt-get clean

# Install ROS dependencies
RUN apt-get install -y ros-${ROS_DISTRO}-desktop-full \
    ros-${ROS_DISTRO}-control-msgs \
    ros-${ROS_DISTRO}-diagnostic-updater \
    ros-${ROS_DISTRO}-controller-interface \
    ros-${ROS_DISTRO}-combined-robot-hw \
    ros-${ROS_DISTRO}-joint-limits-interface \
    ros-${ROS_DISTRO}-libfranka \
    ros-${ROS_DISTRO}-franka-ros 

# Install Pinocchio and its dependencies
RUN apt-get install -y \
    robotpkg-py38-pinocchio \
    robotpkg-pinocchio \
    && apt-get clean

# Set up Pinocchio environment variables
ENV PATH=/opt/openrobots/bin:$PATH
ENV PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH
ENV LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH
ENV PYTHONPATH=/opt/openrobots/lib/python3.8/site-packages:$PYTHONPATH
ENV CMAKE_PREFIX_PATH=/opt/openrobots:$CMAKE_PREFIX_PATH

# Add environment variables to .bashrc and set permissions
RUN echo 'export PATH=/opt/openrobots/bin:$PATH' >> /home/ros/.bashrc && \
    echo 'export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH' >> /home/ros/.bashrc && \
    echo 'export LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH' >> /home/ros/.bashrc && \
    echo 'export PYTHONPATH=/opt/openrobots/lib/python3.8/site-packages:$PYTHONPATH' >> /home/ros/.bashrc && \
    echo 'export CMAKE_PREFIX_PATH=/opt/openrobots:$CMAKE_PREFIX_PATH' >> /home/ros/.bashrc && \
    chown ros:ros /home/ros/.bashrc

# Switch back to ros user and verify Pinocchio installation
USER ${USER}
RUN bash -c "source /home/ros/.bashrc && python3 -c 'import pinocchio as pin; print(\"Pinocchio import successful\")'"

# Install PyTorch with CUDA support - Python 3.8 compatible version
RUN pip3 install --no-cache-dir \
    torch==1.13.1 torchvision==0.14.1 torchaudio==0.13.1 --index-url https://download.pytorch.org/whl/cu116
    
# Handle SIMD option
RUN if [ "${USE_SIMD}" = "ON" ] ; \ 
    then export CMAKE_CXX_FLAGS="-march=native -faligned-new" ; fi

WORKDIR /home/${USER}/ros_ws/src

# Install Virtual Tactile Pad with verification
RUN git clone --depth 1 https://github.com/rayangdn/virtual_tactile_pad.git && \
    cp -R virtual_tactile_pad/ros_ws/src/* . && \
    rm -rf virtual_tactile_pad
    
# Setup ROS workspace
WORKDIR /home/${USER}/ros_ws
RUN rosdep install --from-paths src --ignore-src --rosdistro noetic -y --skip-keys libfranka && \
    source /opt/ros/$ROS_DISTRO/setup.bash && \
    catkin_make -DCMAKE_BUILD_TYPE=Release 

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