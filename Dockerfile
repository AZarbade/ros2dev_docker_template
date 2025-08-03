FROM osrf/ros:jazzy-desktop-full

# Install deps
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

# Create a non-root user with a different UID
ARG USERNAME=ros2dev
ARG USER_UID=1001
ARG USER_GID=1001

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Set up rosdep
USER $USERNAME
RUN rosdep update

# Set workspace
WORKDIR /ros2_ws
RUN mkdir -p src && chown -R $USER_UID:$USER_GID /ros2_ws

# Switch to non-root user
USER $USERNAME

# Source ROS2 in bashrc
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc && \
    echo "if [ -f /ros2_ws/install/setup.bash ]; then source /ros2_ws/install/setup.bash; fi" >> ~/.bashrc

# Create a build script for convenience
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
fi' > /home/$USERNAME/build_ws && chmod +x /home/$USERNAME/build_ws

# Add build script to PATH
RUN echo 'export PATH="/home/'$USERNAME':$PATH"' >> ~/.bashrc

ENTRYPOINT ["/bin/bash"]
