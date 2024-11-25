# Handle ros distro
ARG ROS_DISTRO=noetic

FROM ghcr.io/aica-technology/ros-ws:${ROS_DISTRO}

# Environment setup
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

SHELL ["/bin/bash", "-c"]

# Add user to host GID
ENV USER_GROUP=${USER}
USER root
RUN if [ "HOST_GID" != "1000"] ; \
   then groupadd --gid ${HOST_GID} host_group && \
   usermod ${USER} -g ${HOST_GID} && \ 
   usermod ${USER} -a -G ${USER_GROUP}; fi

# Git config
USER ${USER}
RUN git config --global user.name "${GIT_NAME}" && \
   git config --global user.email "${GIT_EMAIL}"

# Install robotpkg
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
   ros-${ROS_DISTRO}-joint-limits-interface

# Install libfranka and franka_ros from source
WORKDIR /home/${USER}/ros_ws/src
RUN git clone --recursive -b 0.8.0 https://github.com/frankaemika/libfranka && \
   cd libfranka && \
   mkdir build && \
   cd build && \
   cmake -DCMAKE_BUILD_TYPE=Release .. && \
   cmake --build . -j$(nproc) && \
   sudo make install && \
   sudo ldconfig && \
   cd .. && \
   git clone --recursive -b 0.8.0 https://github.com/frankaemika/franka_ros

# Install Pinocchio
RUN apt-get install -y \
   robotpkg-py38-pinocchio \
   robotpkg-pinocchio \
   && apt-get clean

# Configure Pinocchio environment
ENV PATH=/opt/openrobots/bin:$PATH
ENV PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH
ENV LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH
ENV PYTHONPATH=/opt/openrobots/lib/python3.8/site-packages:$PYTHONPATH
ENV CMAKE_PREFIX_PATH=/opt/openrobots:$CMAKE_PREFIX_PATH

# Update bashrc
RUN echo 'export PATH=/opt/openrobots/bin:$PATH' >> /home/ros/.bashrc && \
   echo 'export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH' >> /home/ros/.bashrc && \
   echo 'export LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH' >> /home/ros/.bashrc && \
   echo 'export PYTHONPATH=/opt/openrobots/lib/python3.8/site-packages:$PYTHONPATH' >> /home/ros/.bashrc && \
   echo 'export CMAKE_PREFIX_PATH=/opt/openrobots:$CMAKE_PREFIX_PATH' >> /home/ros/.bashrc && \
   chown ros:ros /home/ros/.bashrc

# Verify Pinocchio
# USER ${USER}
# RUN bash -c "source /home/ros/.bashrc && python3 -c 'import pinocchio as pin; print(\"Pinocchio import successful\")'"

# Install PyTorch
RUN pip3 install --no-cache-dir \
   torch==1.13.1 torchvision==0.14.1 torchaudio==0.13.1 --index-url https://download.pytorch.org/whl/cu116

# Handle SIMD
RUN if [ "${USE_SIMD}" = "ON" ] ; \ 
   then export CMAKE_CXX_FLAGS="-march=native -faligned-new" ; fi

# Install Virtual Tactile Pad
WORKDIR /home/${USER}/ros_ws/src
RUN git clone --depth 1 https://github.com/rayangdn/virtual_tactile_pad.git && \
   cp -R virtual_tactile_pad/ros_ws/src/* . && \
   rm -rf virtual_tactile_pad

# Setup workspace
WORKDIR /home/${USER}/ros_ws
RUN rosdep install --from-paths src --ignore-src --rosdistro noetic -y --skip-keys libfranka && \
   source /opt/ros/$ROS_DISTRO/setup.bash && \
   catkin_make -DCMAKE_BUILD_TYPE=Release 

# Final configuration
WORKDIR /home/${USER}
RUN sudo chown -R ${USER}:${HOST_GID} .bashrc

# Add SIMD to bashrc if needed
RUN if [ "${USE_SIMD}" = "ON" ] ; \
   then echo "export ENABLE_SIMD=ON" >> /home/${USER}/.bashrc ; fi

# Source ROS
RUN echo "source /opt/ros/noetic/setup.bash && \
   source /home/${USER}/ros_ws/devel/setup.bash" >> /home/${USER}/.bashrc

# Runtime directory setup
RUN sudo mkdir -p /tmp/runtime-ros && \
   sudo chmod 0700 /tmp/runtime-ros && \
   sudo chown ${USER}:${USER} /tmp/runtime-ros

# Graphics environment
ENV XDG_RUNTIME_DIR="/tmp/runtime-ros"
ENV LIBGL_ALWAYS_SOFTWARE=1
ENV DEBIAN_FRONTEND=noninteractive

WORKDIR /home/${USER}/ros_ws