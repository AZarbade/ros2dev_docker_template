FROM osrf/ros:jazzy-desktop-full

# Install dependencies
RUN apt update && apt install -y \
    git \
    vim \
    tree \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    build-essential \
    cmake \
    gdb \
    sudo \
    && rm -rf /var/lib/apt/lists/*

# ROS 2 workspace
WORKDIR /ros2_ws
RUN mkdir -p src build install log

# rosdep update (as root, only once)
RUN rosdep update

# Source ROS setup in bashrc for all users
RUN echo "source /opt/ros/jazzy/setup.bash" >> /etc/bash.bashrc && \
    echo "if [ -f /ros2_ws/install/setup.bash ]; then source /ros2_ws/install/setup.bash; fi" >> /etc/bash.bashrc

# Create build script available system-wide
RUN echo '#!/bin/bash\n\
source /opt/ros/jazzy/setup.bash\n\
cd /ros2_ws\n\
if [ "$(ls -A src)" ]; then\n\
    rosdep install --from-paths src --ignore-src -r -y\n\
    colcon build "$@"\n\
    source install/setup.bash\n\
    echo "Build complete!"\n\
else\n\
    echo "No packages found in src directory"\n\
fi' > /usr/local/bin/build_ws && chmod +x /usr/local/bin/build_ws

WORKDIR /ros2_ws
CMD ["/bin/bash"]
