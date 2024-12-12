# Handle ros distro
ARG ROS_DISTRO=noetic

FROM ghcr.io/aica-technology/ros-ws:${ROS_DISTRO}

# User provided arguments
ARG HOST_GID=1001
ENV USER_GROUP=${USER}

# Tell docker we want to use bash instead of sh in general
SHELL ["/bin/bash", "-c"]

# Add the user to the current GID of the host to avoid permission issues in volumes
USER root
RUN if [ "${HOST_GID}" != "1000" ]; \
    then groupadd --gid ${HOST_GID} host_group && \
    usermod ${USER} -g ${HOST_GID} && \ 
    usermod ${USER} -a -G ${USER_GROUP}; fi

# Install PyTorch with specific version compatible with Python 3.8
RUN pip3 install --no-cache-dir \
torch==1.13.1 \
torchvision==0.14.1 \
torchaudio==0.13.1

# Install ROS packages and dependencies
RUN apt-get update -y && \
    apt-get install -y \
    ros-${ROS_DISTRO}-ros-controllers \
    ros-${ROS_DISTRO}-gazebo-ros-control \
    ros-${ROS_DISTRO}-rviz \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-rqt \
    ros-${ROS_DISTRO}-rqt-common-plugins \
    ros-${ROS_DISTRO}-moveit \
    build-essential \
    cmake \
    git \
    libpoco-dev \
    libeigen3-dev \
    python-is-python3 \
    && apt-get clean

# Make sure .bashrc exists and has correct permissions
RUN touch /home/${USER}/.bashrc && \
    chown ${USER}:${HOST_GID} /home/${USER}/.bashrc && \
    chmod 644 /home/${USER}/.bashrc

# Switch to user for git operations
USER ${USER}

# Install and build virtual_tactile_pad
WORKDIR /home/${USER}/ros_ws/src
RUN git clone --recursive --depth 1 https://github.com/rayangdn/virtual_tactile_pad.git && \
    cp -R virtual_tactile_pad/ros_ws/src/* . && \
    cd libfranka && \
    mkdir build && \
    cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF .. && \
    cmake --build . && \
    make -j && \
    sudo make install -j && \
    sudo ldconfig && \
    rm -rf virtual_tactile_pad

WORKDIR /home/${USER}/ros_ws
RUN rosdep install --from-paths src --ignore-src --rosdistro noetic -y --skip-keys libfranka
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash; catkin_make -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/home/${USER}/libfranka/build"

# Setup environment (as root to avoid permission issues)
USER root
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/${USER}/.bashrc && \
    echo "source /home/${USER}/ros_ws/devel/setup.bash" >> /home/${USER}/.bashrc && \
    chown ${USER}:${HOST_GID} /home/${USER}/.bashrc

# Install robotpkg
RUN apt-get update && apt-get install -qqy \
    lsb-release \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Setup robotpkg repository
RUN mkdir -p /etc/apt/keyrings && \
    curl http://robotpkg.openrobots.org/packages/debian/robotpkg.asc | tee /etc/apt/keyrings/robotpkg.asc && \
    echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" \
    | tee /etc/apt/sources.list.d/robotpkg.list

# Install Pinocchio
RUN apt-get update && \
    apt-get install -qqy robotpkg-py3*-pinocchio && \
    rm -rf /var/lib/apt/lists/*

# Setup environment variables for robotpkg
RUN echo 'export PATH=/opt/openrobots/bin:$PATH' >> /home/${USER}/.bashrc && \
    echo 'export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH' >> /home/${USER}/.bashrc && \
    echo 'export LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH' >> /home/${USER}/.bashrc && \
    echo 'export PYTHONPATH=/opt/openrobots/lib/python3.8/site-packages:$PYTHONPATH' >> /home/${USER}/.bashrc && \
    echo 'export CMAKE_PREFIX_PATH=/opt/openrobots:$CMAKE_PREFIX_PATH' >> /home/${USER}/.bashrc && \
    chown ${USER}:${HOST_GID} /home/${USER}/.bashrc

# Switch back to user
USER ${USER}

# Install Python packages
RUN pip3 install --no-cache-dir \
    pandas \
    matplotlib 

# Install tkinter system dependencies and package
RUN sudo apt-get update && \
    sudo apt-get install -y \
    python3-tk 

# Final cleanup
RUN sudo apt-get update && \
    sudo apt-get upgrade -y && \
    sudo apt-get clean

WORKDIR /home/${USER}/ros_ws
# RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash; catkin_make; source devel/setup.bash"
