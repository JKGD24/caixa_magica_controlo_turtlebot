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

### Windows Setup

1. **Download and Install Docker Desktop**  
   - Get **Docker Desktop** from [Docker’s official website](https://www.docker.com/products/docker-desktop/).
   - Follow the installation guide: [Windows Install Guide](https://docs.docker.com/desktop/setup/install/windows-install/).
   - During installation, enable **WSL 2 Backend** (recommended).

2. **Verify Docker Installation**  
   Open **PowerShell** and run:
   
   ```powershell
   docker --version
   ```  
   If Docker is installed correctly, this command will display the installed version.

3. **Enable WSL Integration for Docker**  
   - Open **Docker Desktop** on Windows.  
   - Go to **Settings** ⚙️ (click the gear icon).  
   - Navigate to **Resources > WSL Integration**.  
   - Find your **Ubuntu** distribution and toggle it **ON**.  
   - Click **Apply & Restart**.

4. **Launch WSL and Verify Docker**  
   Open **PowerShell** and start Ubuntu in WSL:
   
   ```powershell
   wsl -d Ubuntu
   ```  
   Once inside WSL, confirm that Docker is accessible:
   
   ```sh
   docker --version
   ```  

   Now you’re ready to use Docker inside WSL! 🚀

## Getting Started

### 1. Clone the Repository

```bash
git clone https://github.com/SamuelTeixeira1/caixinha.git
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
    --name=caixinha_container \
    caixinha
```

To start a container in the background and reattach later:

```bash
docker run -dit --name caixinha \
    --runtime=runc --env=DISPLAY=$DISPLAY \
    --env=QT_X11_NO_MITSHM=1 \
    --volume=/tmp/.X11-unix:/tmp/.X11-unix:rw \
    --volume=$HOME/.ssh:/home/ros/.ssh \
    caixinha
```

On Windows, add this:

```bash
export DISPLAY=:0
```

To enter an already running container:

```bash
docker exec -it caixinha /bin/bash
```

## Run ROS 2 ArduPilot Tests

```bash
cd ~/ardu_ws/
colcon test --packages-select ardupilot_dds_tests
```

## Build ROS 2 Packages

```bash
colcon build --packages-up-to ardupilot_sitl
colcon build --packages-up-to ardupilot_gz_bringup
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


