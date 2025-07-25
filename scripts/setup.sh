#!/bin/bash
# Quick setup script

echo "Setting up ROS2 development environment..."

# Create workspace structure
mkdir -p src scripts

# Set X11 permissions for GUI apps
xhost +local:docker

# Build and start containers
docker-compose build
docker-compose up -d ros2-dev

echo "Environment ready! Use: docker-compose exec ros2-dev bash"
