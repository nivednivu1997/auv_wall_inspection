# AUV Distance Keep Controller
# ROS2 Humble + Gazebo + Nav2

FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive

# Install required packages
RUN apt update && apt install -y \
    build-essential \
    git \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-pytest \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-behaviortree-cpp-v3 \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-rqt \
    ros-humble-rqt-graph \
    ros-humble-rqt-topic \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init || true && rosdep update

# Create workspace
WORKDIR /ws

# Copy source
COPY src/ /ws/src/

# Install dependencies
RUN . /opt/ros/humble/setup.sh && \
    rosdep install --from-paths src --ignore-src -r -y

# Build workspace
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install

# Add entrypoint script
COPY docker-entrypoint.sh /docker-entrypoint.sh
RUN chmod +x /docker-entrypoint.sh

# Source ROS automatically in shell
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /ws/install/setup.bash" >> ~/.bashrc

ENTRYPOINT ["/docker-entrypoint.sh"]
CMD []
