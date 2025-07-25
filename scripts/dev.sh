#!/bin/bash
# Development helper script

case "$1" in
    "start")
        docker-compose up -d ros2-dev
        ;;
    "shell")
        docker-compose exec ros2-dev bash
        ;;
    "build")
        docker-compose exec ros2-dev build_ws
        ;;
    "clean")
        docker-compose exec ros2-dev bash -c "cd /ros2_ws && rm -rf build install log"
        ;;
    "stop")
        docker-compose down
        ;;
    *)
        echo "Usage: $0 {start|shell|build|clean|stop}"
        ;;
esac
