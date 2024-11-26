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

# Install ROS packages and dependencies
RUN apt-get update -y && \
    apt-get install -y \
    ros-${ROS_DISTRO}-ros-controllers \
    ros-${ROS_DISTRO}-gazebo-ros-control \
    ros-${ROS_DISTRO}-rviz \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-rqt \
    ros-${ROS_DISTRO}-rqt-common-plugins \
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

# Install and build libfranka
# WORKDIR /home/${USER}/ros_ws/src
# RUN git clone --recursive https://github.com/frankaemika/libfranka && \
#     cd libfranka && \
#     git checkout 0.8.0 && \
#     git submodule update && \
#     mkdir build && \
#     cd build && \
#     cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF .. && \
#     cmake --build . && \
#     make -j && \
#     sudo make install -j && \
#     sudo ldconfig

# # Install and build franka_ros
# WORKDIR /home/${USER}/ros_ws/src
# RUN git clone --recursive https://github.com/frankaemika/franka_ros --branch noetic-devel
# RUN cd franka_ros && git checkout 0.8.0 && git submodule update

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

# Switch back to user
USER ${USER}

# Final cleanup
RUN sudo apt-get update && \
    sudo apt-get upgrade -y && \
    sudo apt-get clean

WORKDIR /home/${USER}/ros_ws
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash; catkin_make; source devel/setup.bash"