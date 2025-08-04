#!/bin/bash
# ROS2 Development Container Helper Script
# Manages the ROS2 Jazzy development environment

CONTAINER_NAME="ros2-jazzy-dev"
COMPOSE_FILE="docker-compose.yml"

check_container_running() {
    if ! docker ps --format "table {{.Names}}" | grep -q "^${CONTAINER_NAME}$"; then
        return 1
    fi
    return 0
}

show_usage() {
    cat << EOF
ROS2 Development Environment Helper

USAGE:
    \$0 <command> [options]

COMMANDS:
    Environment Management:
        start                   Start the ROS2 development container
        stop                    Stop and remove the container
        restart                 Restart the container
        status                  Show container status
        logs [--follow]         Show container logs (use --follow for live logs)

    Development:
        shell                   Open interactive bash shell in container
        build [package]         Build workspace (optionally specify package)
        test [package]          Run tests (optionally specify package)
        clean                   Clean build artifacts (build/, install/, log/)
        deps                    Install dependencies using rosdep

    ROS2 Commands:
        ros2 <args>            Execute ros2 commands in container
        create <type> <name>   Create new ROS2 package
                              Types: cpp, python, cmake
        list                   List all packages in workspace
        info [package]         Show package information

    Utilities:
        exec <command>         Execute arbitrary command in container
        fix-perms              Fix file permissions for host/container sync
        workspace              Show workspace information
        help                   Show this help message

EXAMPLES:
    \$0 start                           # Start the development environment
    \$0 shell                           # Open interactive shell
    \$0 create cpp my_robot_pkg         # Create new C++ package
    \$0 build my_robot_pkg              # Build specific package
    \$0 ros2 topic list                 # List ROS2 topics
    \$0 exec "cd src && git status"     # Run git command in src directory
    \$0 logs --follow                   # Follow container logs

TIPS:
    • Use '\$0 status' to check if container is running
    • Use '\$0 workspace' to see current workspace structure
    • Source files are mounted at ./src on host
    • Build artifacts are stored in container volumes
EOF
}

case "$1" in
    "start")
        echo "[INFO] Starting ROS2 development container..."
        if check_container_running; then
            echo "[WARNING] Container is already running"
        else
            docker-compose up -d ros2-dev
            if [ $? -eq 0 ]; then
                echo "[SUCCESS] Container started successfully"
                echo "[INFO] Use '$0 shell' to open interactive session"
            else
                echo "[ERROR] Failed to start container"
                exit 1
            fi
        fi
        ;;
    
    "stop")
        echo "[INFO] Stopping ROS2 development container..."
        docker-compose down
        echo "[SUCCESS] Container stopped"
        ;;
    
    "restart")
        echo "[INFO] Restarting ROS2 development container..."
        docker-compose restart ros2-dev
        echo "[SUCCESS] Container restarted"
        ;;
    
    "status")
        if check_container_running; then
            echo "[SUCCESS] Container is running"
            docker ps --filter "name=${CONTAINER_NAME}" --format "table {{.Names}}\t{{.Status}}\t{{.Ports}}"
        else
            echo "[WARNING] Container is not running"
        fi
        ;;
    
    "logs")
        if [ "$2" = "--follow" ] || [ "$2" = "-f" ]; then
            docker-compose logs -f ros2-dev
        else
            docker-compose logs ros2-dev
        fi
        ;;
    
    "shell")
        if ! check_container_running; then
            echo "[ERROR] Container is not running. Use '$0 start' first."
            exit 1
        fi
        echo "[INFO] Opening interactive shell..."
        docker-compose exec ros2-dev bash
        ;;
    
    "build")
        if ! check_container_running; then
            echo "[ERROR] Container is not running. Use '$0 start' first."
            exit 1
        fi
        if [ -n "$2" ]; then
            echo "[INFO] Building package: $2"
            docker-compose exec ros2-dev bash -c "source /opt/ros/jazzy/setup.bash && cd /ros2_ws && colcon build --packages-select $2"
        else
            echo "[INFO] Building entire workspace..."
            docker-compose exec ros2-dev build_ws
        fi
        ;;
    
    "test")
        if ! check_container_running; then
            echo "[ERROR] Container is not running. Use '$0 start' first."
            exit 1
        fi
        if [ -n "$2" ]; then
            echo "[INFO] Testing package: $2"
            docker-compose exec ros2-dev bash -c "source /opt/ros/jazzy/setup.bash && cd /ros2_ws && colcon test --packages-select $2"
        else
            echo "[INFO] Testing entire workspace..."
            docker-compose exec ros2-dev bash -c "source /opt/ros/jazzy/setup.bash && cd /ros2_ws && colcon test"
        fi
        ;;
    
    "clean")
        if ! check_container_running; then
            echo "[ERROR] Container is not running. Use '$0 start' first."
            exit 1
        fi
        echo "[INFO] Cleaning build artifacts..."
        docker-compose exec ros2-dev bash -c "cd /ros2_ws && rm -rf build install log"
        echo "[SUCCESS] Workspace cleaned"
        ;;
    
    "deps")
        if ! check_container_running; then
            echo "[ERROR] Container is not running. Use '$0 start' first."
            exit 1
        fi
        echo "[INFO] Installing dependencies..."
        docker-compose exec ros2-dev bash -c "source /opt/ros/jazzy/setup.bash && cd /ros2_ws && rosdep install --from-paths src --ignore-src -r -y"
        ;;
    
    "create")
        if ! check_container_running; then
            echo "[ERROR] Container is not running. Use '$0 start' first."
            exit 1
        fi
        if [ -z "$2" ] || [ -z "$3" ]; then
            echo "[ERROR] Usage: $0 create <type> <package_name>"
            echo "Types: cpp, python, cmake"
            exit 1
        fi
        case "$2" in
            "cpp")
                BUILD_TYPE="ament_cmake"
                ;;
            "python")
                BUILD_TYPE="ament_python"
                ;;
            "cmake")
                BUILD_TYPE="ament_cmake"
                ;;
            *)
                echo "[ERROR] Invalid package type. Use: cpp, python, or cmake"
                exit 1
                ;;
        esac
        echo "[INFO] Creating $2 package: $3"
        docker-compose exec ros2-dev bash -c "cd /ros2_ws/src && ros2 pkg create --build-type $BUILD_TYPE $3"
        ;;
    
    "list")
        if ! check_container_running; then
            echo "[ERROR] Container is not running. Use '$0 start' first."
            exit 1
        fi
        echo "[INFO] Packages in workspace:"
        docker-compose exec ros2-dev bash -c "source /opt/ros/jazzy/setup.bash && cd /ros2_ws && find src -name 'package.xml' -exec dirname {} \; | sed 's|src/||' | sort"
        ;;
    
    "info")
        if ! check_container_running; then
            echo "[ERROR] Container is not running. Use '$0 start' first."
            exit 1
        fi
        if [ -n "$2" ]; then
            echo "[INFO] Package information for: $2"
            docker-compose exec ros2-dev bash -c "source /opt/ros/jazzy/setup.bash && ros2 pkg xml $2"
        else
            echo "[INFO] Workspace information:"
            docker-compose exec ros2-dev bash -c "cd /ros2_ws && find src -name 'package.xml' | wc -l | xargs echo 'Total packages:'"
        fi
        ;;
    
    "ros2")
        if ! check_container_running; then
            echo "[ERROR] Container is not running. Use '$0 start' first."
            exit 1
        fi
        shift
        docker-compose exec ros2-dev bash -c "source /opt/ros/jazzy/setup.bash && ros2 $*"
        ;;
    
    "exec")
        if ! check_container_running; then
            echo "[ERROR] Container is not running. Use '$0 start' first."
            exit 1
        fi
        if [ -z "$2" ]; then
            echo "[ERROR] Usage: $0 exec \"<command>\""
            exit 1
        fi
        shift
        docker-compose exec ros2-dev bash -c "$*"
        ;;
    
    "fix-perms")
        echo "[INFO] Fixing file permissions..."
        sudo chown -R $(id -u):$(id -g) ./src
        echo "[SUCCESS] Permissions fixed for ./src"
        ;;
    
    "workspace")
        echo "[INFO] Workspace structure:"
        echo "Host directory: $(pwd)"
        echo "Container workspace: /ros2_ws"
        echo "Source mount: ./src -> /ros2_ws/src"
        if [ -d "./src" ]; then
            echo ""
            echo "Source packages:"
            find ./src -maxdepth 2 -name "package.xml" -exec dirname {} \; | sed 's|./src/|  - |' | sort
        else
            echo ""
            echo "No src directory found"
        fi
        ;;
    
    ""|"help"|"--help"|"-h")
        show_usage
        ;;
    
    *)
        echo "[ERROR] Unknown command: $1"
        echo ""
        show_usage
        exit 1
        ;;
esac
