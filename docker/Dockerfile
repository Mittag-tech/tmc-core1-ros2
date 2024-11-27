# Use Ubuntu 24.04 as base image
FROM ubuntu:24.04

# Install dependencies
RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    python3-pip \
    build-essential \
    python3-venv \
    locales \
    software-properties-common \
    terminator \
    && rm -rf /var/lib/apt/lists/* 
RUN mkdir -p /root/.config/terminator
RUN touch /root/.config/terminator/config

# Set up a virtual environment
RUN python3 -m venv /opt/venv
ENV PATH="/opt/venv/bin:$PATH"
RUN locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# Upgrade pip and install colcon-common-extensions
RUN pip install --upgrade pip
RUN pip install -U colcon-common-extensions

# Install ROS 2 Jazzy packages and rosdep
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN sh -c 'echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null'
RUN apt-get update && \
    apt-get install -y ros-jazzy-desktop \
    python3-rosdep \
    ros-jazzy-vision-opencv \
    ros-jazzy-usb-cam && \
    rm -rf /var/lib/apt/lists/*
RUN pip install opencv-contrib-python==4.10.0.84

# Initialize rosdep
RUN rosdep init && rosdep update

# Set environment variables
ENV ROS_DISTRO=jazzy
ENV ROS_ROOT=/opt/ros/$ROS_DISTRO
RUN sh -c 'echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc'

# Set up entrypoint
COPY docker/setup_ros2_jazzy.sh /
RUN chmod +x /setup_ros2_jazzy.sh
ENTRYPOINT ["/setup_ros2_jazzy.sh"]
