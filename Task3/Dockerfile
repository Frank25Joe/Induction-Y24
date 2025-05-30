# Base image with ROS 2 Humble
FROM ros:humble

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# Update and install dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    build-essential \
    git \
    && rm -rf /var/lib/apt/lists/*

# Create workspace directory
WORKDIR /ros2_ws

# Copy only the source folder to avoid build cache issues
COPY src /ros2_ws/src

# Initialize rosdep and install dependencies
RUN rm -f /etc/ros/rosdep/sources.list.d/20-default.list || true && \
    rosdep init && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build

# Source the workspace and set entrypoint
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

ENTRYPOINT ["/bin/bash"]
