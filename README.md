# ROS2 Docker Development Template

A streamlined Docker-based development environment for ROS2 projects that can be easily integrated into any project structure.

> ⚠️ **Highly experimental!** Only use after understanding what this project is doing.  
> Developed on **Arch Linux**; other operating systems not tested.

## Features

- **Project Agnostic**: Works in any directory structure - just drop it where you need ROS2 development
- **Isolated Environments**: Each project gets its own containerized workspace
- **Zero Host Dependencies**: Only requires Docker and Docker Compose
- **Live Development**: Source code mounted for real-time editing
- **GUI Support**: X11 forwarding for RViz, Gazebo, and other GUI tools
- **ROS2 Jazzy**: Latest LTS distribution with full desktop installation

## Quick Start

```bash
# Navigate to your desired development location
cd path/to/your/project/ros-component

# Get the setup script
curl -o setup.sh https://raw.githubusercontent.com/AZarbade/ros2dev_docker_template/master/setup.sh
chmod +x setup.sh

# Initialize ROS2 development environment
./setup.sh init my_project

# Start the container
./setup.sh start

# Open development shell
./setup.sh shell
```

## Available Commands

| Command | Description |
|---------|-------------|
| `init <name>` | Initialize new ROS2 project |
| `start` | Start the development container |
| `stop` | Stop and remove the container |
| `restart` | Restart the container |
| `remove` | Completely remove container and volumes |
| `shell` | Open interactive bash shell |
| `build [package]` | Build workspace or specific package |
| `test [package]` | Run tests for workspace or specific package |
| `clean` | Clean build artifacts |
| `exec "<command>"` | Execute arbitrary command in container |
| `status` | Show project and container status |
| `info` | Show project information |
| `logs [-f]` | Show container logs |

## Project Structure

After initialization, your directory will contain:
```
your-location/
├── .ros2dev/           # Template files (auto-downloaded)
├── src/                # Your ROS2 packages go here
├── setup.sh            # Management script
├── Dockerfile          # Container definition
├── docker-compose.yml  # Service configuration
├── .env               # Project variables
└── .gitignore         # Git ignore rules
```

## Requirements

- Docker
- Docker Compose
- Git (for template download)
- X11 server (for GUI applications)

## Use Cases

Perfect for:
- Multi-component robotics projects
- Academic research with mixed tech stacks  
- Projects where ROS2 is just one component
- Teams needing consistent development environments
- Rapid prototyping and experimentation

---

**License**: MIT | **ROS2 Version**: Jazzy Jalopy
