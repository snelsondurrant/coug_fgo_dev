# Created by Nelson Durrant, Nov 2025
FROM ros:humble-ros-base

ARG DEBIAN_FRONTEND=noninteractive
ARG TARGETARCH

# Define a username and password
ARG NAME=frostlab-docker
ARG PASS=frostlab

# Create the user
RUN useradd -ms /bin/bash "$NAME" \
    && usermod -aG sudo "$NAME" \
    && echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers \
    && echo "$NAME:$PASS" | chpasswd

#################################
# IMPORTANT DOCUMENTATION NOTE! #
#################################

# When adding dependencies or Dockerfile commands below, please document your changes
# following this format:
#
#     # Why do we need this dependency or command? - Full Name, Month Year
#     < Add system dependencies here >
#
# https://docs.docker.com/reference/dockerfile/

# Install system dependencies
RUN apt update && apt upgrade -y --no-install-recommends \
    && apt install -y --no-install-recommends \
    # Install basic system utilities - Nelson Durrant, Nov 2024
    curl wget git vim nano tmux python3-pip python3-venv \
    # Install graphical utilities - Nelson Durrant, Nov 2024
    x11-apps x11-utils x11-xserver-utils xauth \
    && rm -rf /var/lib/apt/lists/*

# Install GTSAM (build from source for ARM compatibility) - Nelson Durrant, Nov 2025
RUN apt update && apt install -y --no-install-recommends \
    build-essential cmake libboost-dev libtbb-dev \
    libboost-serialization-dev \
    libboost-system-dev \
    libboost-filesystem-dev \
    libboost-thread-dev \
    libboost-program-options-dev \
    libboost-date-time-dev \
    libboost-timer-dev \
    libboost-chrono-dev \
    libboost-regex-dev \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /home/$NAME
RUN git clone --depth 1 --branch 4.2.0 https://github.com/borglab/gtsam.git \
    && cd gtsam \
    && mkdir build && cd build \
    && cmake .. -DCMAKE_BUILD_TYPE=Release -DGTSAM_WITH_TBB=ON \
    && make -j$(nproc) \
    && sudo make install \
    && sudo ldconfig \
    && cd /home/$NAME \
    && rm -rf gtsam

# Install ROS packages
RUN apt update && apt install -y --no-install-recommends \
    # Robot localization package for comparison - Nelson Durrant, Nov 2025
    ros-humble-robot-localization \
    # Used for GUIs - Nelson Durrant, Nov 2025
    ros-humble-rviz2 ros-humble-rqt* \
    # Tool for plotting and visualizing data from ROS2 bags - Nelson Durrant, Nov 2025
    ros-humble-plotjuggler-ros \
    # Controlling the robot's movement through a joystick or keyboard - Nelson Durrant, Nov 2025
    ros-humble-joy ros-humble-teleop-twist-joy ros-humble-teleop-twist-keyboard ros-humble-topic-tools \
    # ROS2 and tf2 tutorial installs - Nelson Durrant, Nov 2025
    ros-humble-turtle-tf2-py ros-humble-tf2-ros ros-humble-tf2-tools ros-humble-turtlesim \
    # URDF tools and launch configs - Nelson Durrant, Nov 2025
    ros-humble-xacro ros-humble-joint-state-publisher-gui ros-humble-urdf-launch \
    # UTM conversion tool - Nelson Durrant, Nov 2025
    ros-humble-geodesy \
    && rm -rf /var/lib/apt/lists/*

# Install architecture-specific ROS packages
RUN if [ "$TARGETARCH" = "amd64" ]; then \
    apt update && apt install -y --no-install-recommends \
    # Tools for visualizing the robot's location - Nelson Durrant, Nov 2025
    ros-humble-mapviz ros-humble-mapviz-plugins ros-humble-tile-map \
    && rm -rf /var/lib/apt/lists/*; \
    fi

USER $NAME
WORKDIR /home/$NAME

# Install python dependencies
RUN pip3 install --no-cache-dir \
    # Used for ROS2 bag plotting - Nelson Durrant, Nov 2025
    rosbags
ENV PATH="$PATH:/home/$NAME/.local/bin"

# Set up automatic ROS2 sourcing and colorized output - Nelson Durrant, Nov 2024
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc \
    && echo "export RCUTILS_COLORIZED_OUTPUT=1" >> ~/.bashrc \
    && touch ~/.hushlogin

CMD ["bash"]
