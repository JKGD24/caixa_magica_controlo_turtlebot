# Caixinha: ROS2 + SITL + Gazebo + ArduPilot in Docker

This repository sets up a complete Docker environment for running ArduPilot SITL (Software In The Loop) with ROS2. It is designed to provide a robust and flexible development environment, allowing seamless integration between ArduPilot, Gazebo, and ROS2.

## Prerequisites

Before using this setup, ensure that you have the following installed on your system:

- **Docker**: Install Docker using the instructions below.
- **X11 for GUI applications**: Required to run graphical applications inside the container (e.g., QGroundControl, Gazebo).

### Install Docker

#### Linux (Tested on Ubuntu 22.04)

```bash
sudo apt update
sudo apt install -y ca-certificates curl gnupg
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo tee /etc/apt/keyrings/docker.gpg > /dev/null
sudo chmod a+r /etc/apt/keyrings/docker.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt update
sudo apt install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```

To enable Docker without `sudo` (optional):
```bash
sudo groupadd docker
sudo usermod -aG docker $USER
newgrp docker
```

#### Windows

1. Download and install **Docker Desktop** from [Docker's official website](https://www.docker.com/products/docker-desktop/).
2. Enable **WSL 2 Backend** (recommended) during installation.
3. Restart your system after installation.
4. Open PowerShell and verify installation:
   ```powershell
   docker --version
   ```

## Getting Started

### 1. Clone the Repository
```bash
git clone https://github.com/yourusername/caixinha.git
cd caixinha
```

### 2. Build the Docker Image
```bash
docker build --network=host -t caixinha .
```

### 3. Run the Container
```bash
docker run -it --rm --runtime=runc \
    --interactive --env=DISPLAY=$DISPLAY \
    --env=QT_X11_NO_MITSHM=1 \
    --volume=/tmp/.X11-unix:/tmp/.X11-unix:rw \
    --volume=$HOME/.ssh:/home/ros/.ssh \
    --name=caixinha_container
    caixinha
```

To start a container in the background and reattach later:
```bash
docker run -dit --name caixinha \
    --runtime=runc --env=DISPLAY=$DISPLAY \
    --env=QT_X11_NO_MITSHM=1 \
    --volume=/tmp/.X11-unix:/tmp/.X11-unix:rw \
    --volume=$HOME/.ssh:/home/ros/.ssh \
    --name=caixinha_container
    caixinha
```

To enter an already running container:
```bash
docker exec -it caixinha /bin/bash
```

## Testing the Setup

### Verify ROS2 Installation
```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker
```
In another terminal:
```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_py listener
```

### Run SITL Simulation
```bash
cd ~/ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter --console --map -w
```

### Run Gazebo Simulation
```bash
source install/setup.bash
ros2 launch ardupilot_gz_bringup iris_runway.launch.py
```

### Test QGroundControl
```bash
cd ~
./QGroundControl.AppImage
# Extract QGroundControl AppImage
./QGroundControl.AppImage --appimage-extract
./squashfs-root/QGroundControl
```

## Additional Commands
Refer to [commands.txt](commands.txt) and [tests.txt](tests.txt) for more setup details.

## Contributions
Contributions are welcome! Feel free to open an issue or submit a pull request.


