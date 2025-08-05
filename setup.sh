#!/bin/bash
# ROS2 Development Environment Manager
# Handles project initialization and development workflow

set -e

# Configuration
DEFAULT_PROJECT_NAME=$(basename "$(pwd)")
ENV_FILE=".env"
DOCKERFILE="Dockerfile"
COMPOSE_FILE="docker-compose.yml"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# Helper functions
log_info() { echo -e "${BLUE}[INFO]${NC} $1"; }
log_success() { echo -e "${GREEN}[SUCCESS]${NC} $1"; }
log_warning() { echo -e "${YELLOW}[WARNING]${NC} $1"; }
log_error() { echo -e "${RED}[ERROR]${NC} $1"; }

# Check if project is initialized
is_project_initialized() {
    [ -f "$ENV_FILE" ] && [ -f "$DOCKERFILE" ] && [ -f "$COMPOSE_FILE" ]
}

# Get project name from .env file
get_project_name() {
    if [ -f "$ENV_FILE" ]; then
        grep "^PROJECT_NAME=" "$ENV_FILE" | cut -d'=' -f2
    else
        echo "$DEFAULT_PROJECT_NAME"
    fi
}

# Check if container is running
check_container_running() {
    local project_name=$(get_project_name)
    local container_name="ros2-${project_name}-dev"
    if ! docker ps --format "table {{.Names}}" | grep -q "^${container_name}$"; then
        return 1
    fi
    return 0
}

# Initialize project
init_project() {
    local project_name="$1"
    
    if [ -z "$project_name" ]; then
        log_error "Project name required"
        echo "Usage: $0 init <project_name>"
        exit 1
    fi
    
    if is_project_initialized; then
        log_warning "Project already initialized"
        log_info "Current project: $(get_project_name)"
        return 0
    fi
    
    log_info "Initializing ROS2 project: $project_name"
    
    # Create .env file
    cat > "$ENV_FILE" << EOF
PROJECT_NAME=$project_name
COMPOSE_PROJECT_NAME=$project_name
USER_UID=$(id -u)
USER_GID=$(id -g)
EOF

    # Check if templates exist locally first, then try to download/setup
    if [ ! -f "$DOCKERFILE" ] || [ ! -f "$COMPOSE_FILE" ]; then
        if [ ! -d ".ros2dev" ]; then
            log_info "Setting up ROS2 dev template..."
            # Simple clone approach
            git clone https://github.com/AZarbade/ros2dev_docker_template.git .ros2dev || {
                log_error "Failed to download ROS2 dev template. Please ensure git is available and you have internet access."
                exit 1
            }
            rm -rf .ros2dev/.git
        fi
        
        # Copy Dockerfile if missing
        if [ ! -f "$DOCKERFILE" ]; then
            cp .ros2dev/Dockerfile "$DOCKERFILE" || {
                log_error "Failed to copy Dockerfile from .ros2dev/"
                exit 1
            }
        fi
        
        # Copy docker-compose.yml if missing  
        if [ ! -f "$COMPOSE_FILE" ]; then
            cp .ros2dev/docker-compose.yml "$COMPOSE_FILE" || {
                log_error "Failed to copy docker-compose.yml from .ros2dev/"
                exit 1
            }
        fi
    fi

    # Create .gitignore
    cat > .gitignore << 'EOF'
# ROS2 build artifacts
build/
install/
log/
# Docker
.env
EOF
    
    # Create src directory
    mkdir -p src
    
    log_success "Project '$project_name' initialized successfully!"
    log_info "Next steps:"
    echo "  1. ./setup.sh start    # Start the development environment"
    echo "  2. ./setup.sh shell    # Open interactive shell"
    echo "  3. ./setup.sh help     # See all available commands"
}

# Setup/start environment
start_environment() {
    if ! is_project_initialized; then
        log_error "Project not initialized. Run: $0 init <project_name>"
        exit 1
    fi
    
    local project_name=$(get_project_name)
    log_info "Starting ROS2 development environment for: $project_name"
    
    if check_container_running; then
        log_warning "Container is already running"
        return 0
    fi
    
    # Export environment variables
    export $(cat "$ENV_FILE" | xargs)
    
    # Set X11 permissions for GUI apps
    xhost +local:docker >/dev/null 2>&1 || log_warning "Could not set X11 permissions"
    
    # Build and start container
    docker-compose build
    docker-compose up -d ros2-dev
    
    if [ $? -eq 0 ]; then
        log_success "Development environment started"
        log_info "Use '$0 shell' to open interactive session"
    else
        log_error "Failed to start development environment"
        exit 1
    fi
}

# Show project information
show_project_info() {
    if ! is_project_initialized; then
        log_error "Project not initialized"
        exit 1
    fi
    
    local project_name=$(get_project_name)
    log_info "Project Information"
    echo "Name: $project_name"
    echo "Directory: $(pwd)"
    echo "Docker files: Dockerfile, docker-compose.yml"
    echo "Environment file: .env"
    
    if [ -d "./src" ]; then
        echo "Source packages:"
        find ./src -maxdepth 2 -name "package.xml" -exec dirname {} \; 2>/dev/null | sed 's|./src/|  - |' | sort || echo "  (none found)"
    else
        echo "No src directory found"
    fi
}

# Show usage help
show_usage() {
    cat << EOF
ROS2 Development Environment Manager

USAGE:
    $0 <command> [options]

PROJECT MANAGEMENT:
    init <name>             Initialize new ROS2 project
    status                  Show project and container status
    info                    Show project information

ENVIRONMENT MANAGEMENT:
    start                   Start the development container
    stop                    Stop the container
    restart                 Restart the container
    remove                  Completely remove container and volumes
    logs [--follow]         Show container logs

DEVELOPMENT:
    shell                   Open interactive bash shell in container
    build [package]         Build workspace (optionally specify package)
    test [package]          Run tests (optionally specify package)
    clean                   Clean build artifacts

UTILITIES:
    exec <command>          Execute arbitrary command in container
    help                    Show this help message

EXAMPLES:
    $0 init my_robot                    # Initialize new project
    $0 start                            # Start development environment
    $0 build navigation                 # Build specific package
    $0 remove                           # Completely remove container
    $0 exec "cd src && git status"      # Run git commands

TIPS:
    • Each project gets isolated containers and volumes
    • Source files are mounted at ./src for live editing
    • Build artifacts persist in Docker volumes
    • Use '$0 status' to check current state
EOF
}

# Main command handling
case "$1" in
    "init")
        init_project "$2"
        ;;
    
    "start"|"setup")
        start_environment
        ;;
    
    "stop")
        if ! is_project_initialized; then
            log_error "Project not initialized"
            exit 1
        fi
        export $(cat "$ENV_FILE" | xargs)
        log_info "Stopping development environment..."
        docker-compose down
        log_success "Environment stopped"
        ;;
    
    "restart")
        if ! is_project_initialized; then
            log_error "Project not initialized"
            exit 1
        fi
        export $(cat "$ENV_FILE" | xargs)
        log_info "Restarting development environment..."
        docker-compose restart ros2-dev
        log_success "Environment restarted"
        ;;
    
    "remove")
        if ! is_project_initialized; then
            log_error "Project not initialized"
            exit 1
        fi
        
        project_name=$(get_project_name)
        log_warning "This will completely remove the container and all associated volumes for project: $project_name"
        echo "This action cannot be undone. All build artifacts and container data will be lost."
        read -p "Are you sure you want to continue? (y/N): " -n 1 -r
        echo
        
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            export $(cat "$ENV_FILE" | xargs)
            
            log_info "Stopping and removing container..."
            docker-compose down
            
            log_info "Removing container image..."
            docker rmi "${project_name}_ros2-dev" 2>/dev/null || log_warning "Image not found or already removed"
            
            log_info "Removing associated volumes..."
            docker volume rm "${project_name}_bash_history" 2>/dev/null || true
            docker volume rm "${project_name}_fish_history" 2>/dev/null || true
            
            # Remove any dangling volumes that might be associated with this project
            docker volume prune -f >/dev/null 2>&1 || true
            
            log_success "Container and volumes completely removed"
            log_info "Project files in ./src are preserved"
            log_info "Run '$0 start' to recreate the development environment"
        else
            log_info "Operation cancelled"
        fi
        ;;
    
    "status")
        if ! is_project_initialized; then
            echo "Status: Not initialized"
            echo "Run: $0 init <project_name> to get started"
            exit 0
        fi
        
        local project_name=$(get_project_name)
        echo "Project: $project_name"
        echo "Directory: $(pwd)"
        
        if check_container_running; then
            echo "Container: Running"
            docker ps --filter "name=ros2-${project_name}-dev" --format "table {{.Names}}\t{{.Status}}"
        else
            echo "Container: Stopped"
        fi
        ;;
    
    "info")
        show_project_info
        ;;
    
    "logs")
        if ! is_project_initialized; then
            log_error "Project not initialized"
            exit 1
        fi
        export $(cat "$ENV_FILE" | xargs)
        if [ "$2" = "--follow" ] || [ "$2" = "-f" ]; then
            docker-compose logs -f ros2-dev
        else
            docker-compose logs ros2-dev
        fi
        ;;
    
    "shell")
        if ! is_project_initialized; then
            log_error "Project not initialized"
            exit 1
        fi
        if ! check_container_running; then
            log_error "Container not running. Use '$0 start' first."
            exit 1
        fi
        export $(cat "$ENV_FILE" | xargs)
        log_info "Opening interactive shell..."
        docker-compose exec ros2-dev bash
        ;;
    
    "build")
        if ! is_project_initialized; then
            log_error "Project not initialized"
            exit 1
        fi
        if ! check_container_running; then
            log_error "Container not running. Use '$0 start' first."
            exit 1
        fi
        export $(cat "$ENV_FILE" | xargs)
        if [ -n "$2" ]; then
            log_info "Building package: $2"
            docker-compose exec ros2-dev bash -c "source /opt/ros/jazzy/setup.bash && cd /ros2_ws && colcon build --packages-select $2"
        else
            log_info "Building entire workspace..."
            docker-compose exec ros2-dev build_ws
        fi
        ;;
    
    "test")
        if ! is_project_initialized; then
            log_error "Project not initialized"
            exit 1
        fi
        if ! check_container_running; then
            log_error "Container not running. Use '$0 start' first."
            exit 1
        fi
        export $(cat "$ENV_FILE" | xargs)
        if [ -n "$2" ]; then
            log_info "Testing package: $2"
            docker-compose exec ros2-dev bash -c "source /opt/ros/jazzy/setup.bash && cd /ros2_ws && colcon test --packages-select $2"
        else
            log_info "Testing entire workspace..."
            docker-compose exec ros2-dev bash -c "source /opt/ros/jazzy/setup.bash && cd /ros2_ws && colcon test"
        fi
        ;;
    
    "clean")
        if ! is_project_initialized; then
            log_error "Project not initialized"
            exit 1
        fi
        if ! check_container_running; then
            log_error "Container not running. Use '$0 start' first."
            exit 1
        fi
        export $(cat "$ENV_FILE" | xargs)
        log_info "Cleaning build artifacts..."
        docker-compose exec ros2-dev bash -c "cd /ros2_ws && rm -rf build install log"
        log_success "Workspace cleaned"
        ;;
    
    "exec")
        if ! is_project_initialized; then
            log_error "Project not initialized"
            exit 1
        fi
        if ! check_container_running; then
            log_error "Container not running. Use '$0 start' first."
            exit 1
        fi
        if [ -z "$2" ]; then
            log_error "Usage: $0 exec \"<command>\""
            exit 1
        fi
        export $(cat "$ENV_FILE" | xargs)
        shift
        docker-compose exec ros2-dev bash -c "$*"
        ;;
    
    "help"|"--help"|"-h"|"")
        show_usage
        ;;
    
    *)
        log_error "Unknown command: $1"
        echo ""
        show_usage
        exit 1
        ;;
esac
