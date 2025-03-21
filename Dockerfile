# Created By Sam
# 20-03-2025

# Use ROS Humble desktop as the base image
FROM osrf/ros:humble-desktop

# Install necessary programs
RUN apt-get update && apt-get install -y \
    nano vim git curl lsb-release gnupg sudo python3-colcon-common-extensions \
    ros-humble-gazebo-* \
    ros-humble-cartographer ros-humble-cartographer-ros \
    ros-humble-navigation2 ros-humble-nav2-bringup \
    && rm -rf /var/lib/apt/lists/*

# ...existing code...

# Install GUI-related dependencies
RUN apt-get update && apt-get install -y \
    x11-apps \
    mesa-utils \
    && rm -rf /var/lib/apt/lists/*

# ...existing code...

RUN apt-get update && apt-get install -y \
libxcb1-dev \
libxcb-keysyms1-dev \
libxcb-image0-dev \
libxcb-shm0-dev \
libxcb-icccm4-dev \
libxcb-shape0-dev \
libxcb-xfixes0-dev \
libxcb-render-util0-dev \
libxcb-randr0-dev \
libxcb-xinerama0-dev

# Create a non-root user
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Switch to the new user
USER $USERNAME
WORKDIR /home/$USERNAME

# Install TurtleBot3 Packages
ENV ROS_WS=/home/$USERNAME/turtlebot3_ws
RUN mkdir -p $ROS_WS/src && cd $ROS_WS/src/ \
    && git clone -b humble-devel https://github.com/ROBOTIS-GIT/DynamixelSDK.git \
    && git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git \
    && git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3.git \
    && git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git

WORKDIR $ROS_WS
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install --parallel-workers 2"

# Environment Configuration
RUN echo 'source ~/turtlebot3_ws/install/setup.bash' >> ~/.bashrc \
    && echo 'export ROS_DOMAIN_ID=30 #TURTLEBOT3' >> ~/.bashrc \
    && echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc \
    && echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc

# Copy the entrypoint and bashrc scripts
COPY --chmod=755 entrypoint.sh /entrypoint.sh
COPY bashrc /home/${USERNAME}/bashrc_custom
RUN cat /home/${USERNAME}/bashrc_custom >> /home/${USERNAME}/.bashrc && rm /home/${USERNAME}/bashrc_custom

# Set up entrypoint and default command
ENTRYPOINT ["/bin/bash", "/entrypoint.sh"]
CMD ["/bin/bash"]

