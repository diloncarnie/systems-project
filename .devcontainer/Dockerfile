FROM ros:humble-ros-base

ARG USERNAME=rosdev
ARG UID=1000
ARG GID=$UID

# Install some dependencies packages
RUN apt update -q \
    && apt upgrade -q -y \
    && apt install -y --no-install-recommends \
    software-properties-common \
    python3-pip \
    xauth \
    i2c-tools \
    libi2c-dev \
    libopencv-dev \
    ros-humble-cv-bridge \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-tf-transformations \
    ros-humble-usb-cam \
    ros-humble-rviz2 \
    libogre1.12.10 \
    && apt clean \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

RUN pip3 install opencv-python \
    smbus \
    easydict \
    gpiozero \
    lgpio

# Create and switch to user
RUN groupadd -g $GID $USERNAME \
    && useradd -lm -u $UID -g $USERNAME -s /bin/bash $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
    
# USER $USERNAME
# RUN sudo chmod a+rw /dev/i2c-*

# Create workspace so that user own this directory
RUN mkdir -p /home/$USERNAME/ros2_ws/src
WORKDIR /home/$USERNAME/ros2_ws

# Copy configuration files
RUN echo 'source /opt/ros/'$ROS_DISTRO'/setup.bash' >> /home/$USERNAME/.bashrc \
    && echo 'source /home/'$USERNAME'/ros2_ws/install/local_setup.bash' >> /home/$USERNAME/.bashrc \
    && echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> /home/$USERNAME/.bashrc  \
    && echo 'export ROS_DOMAIN_ID=5' >> /home/$USERNAME/.bashrc 

# Setup entrypoint
COPY ros_entrypoint.sh /ros_entrypoint.sh
ENTRYPOINT ["/bin/bash","/ros_entrypoint.sh"]
CMD ["bash"]
