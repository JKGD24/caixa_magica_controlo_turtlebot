# Caixinha: ROS2 + SITL + Gazebo + ArduPilot in Docker

This repository sets up a complete Docker environment for running ArduPilot SITL (Software In The Loop) with ROS2. It is designed to provide a robust and flexible development environment, allowing seamless integration between ArduPilot, Gazebo, and ROS2.

## Prerequisites

Before using this setup, ensure that you have the following installed on your system:

- **Docker**: Install Docker using the instructions below.
- **X11 for GUI applications**: Required to run graphical applications inside the container (e.g., QGroundControl, Gazebo).

### Install Docker

#### Linux (Tested on Ubuntu 22.04) (recommended)


Add Docker's official GPG key:
```bash
   sudo apt-get update
   sudo apt-get install ca-certificates curl
   sudo install -m 0755 -d /etc/apt/keyrings
   sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
   sudo chmod a+r /etc/apt/keyrings/docker.asc
```

Add the repository to Apt sources:

```bash
   echo \
      "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
      $(. /etc/os-release && echo "${UBUNTU_CODENAME:-$VERSION_CODENAME}") stable" | \
      sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
   sudo apt-get update
```

Install Docker and its plugins:

```bash
   sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```

To enable Docker without `sudo`:

```bash
   sudo groupadd docker
   sudo usermod -aG docker $USER
   newgrp docker
```
Configure the Atlas Docker Container:

```bash
   echo -e "PUID=$(id -u)\nPGID=$(id -g)\nTZ=Europe/Lisbon" > .env
```

If no `.env` file is created, check permissions, create the file manually and then run it again.

In the docker compose file, change/add the volumes mounted. On the left, put the host machine's directory. On the right, the directory you want the volume to be inside the docker container.

```yaml
   services:
   atlas:
      image: samuelteixeira1/caixinha:latest
      container_name: atlas
      restart: unless-stopped
      volumes:
         - /home/dinis/Desktop/Universidade/Nucleos/ATLAS/:/home/ros/atlas
      environment:
         - PUID=${PUID}
         - PGID=${PGID}
         - TZ=${TZ}
      stdin_open: true
      tty: true 
```

In this case, the host's folder on `/home/dinis/Desktop/Universidade/Nucleos/ATLAS/` is mounted to `/home/ros/atlas`.

Now that the docker container is configured, it's time to build it.

```bash
   docker compose up -d
```

Make sure you are on the same folder where the `compose.yaml` file is. The first time you run it, it will take a while pulling the image. All other times should be almost instantaneous.

Now the container should be up and running. To enter and execute commands in it, use:

```bash
   docker exec -it atlas bash  
```

When your work is done, do the following.

```bash
   exit
   docker compose down  
```


**WARNING: STORE ALL THE DATA INSIDE THE MOUNTED VOLUMES, OTHERWISE IT WILL BE ERASED. IF YOU DON'T WANT TO BE CAREFUL, JUST STOP INSTEAD OF DOWNING THE CONTAINER. JUST BE WARNED IT WILL BE CONSUMING RESOURCES IN THE BACKGROUND.**


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

### Option 1: Pull Pre-Built Docker Image

If you want to skip building the image manually, you can pull the latest pre-built image from Docker Hub:

```bash
docker pull samuelteixeira1/caixinha:latest
```

To run the container:

```bash
docker run -it --runtime=runc \
    --interactive --env=DISPLAY=$DISPLAY \
    --env=QT_X11_NO_MITSHM=1 \
    --volume=/tmp/.X11-unix:/tmp/.X11-unix:rw \
    --volume=$HOME/.ssh:/home/ros/.ssh \
    --name=caixinha_container \
    samuelteixeira1/caixinha:latest
```

### Option 2: Build the Image Manually

#### 1. Clone the Repository

```bash
git clone https://github.com/SamuelTeixeira1/caixinha.git
cd caixinha
```

#### 2. Build the Docker Image

```bash
docker build --network=host -t caixinha .
```

### 3. Run the Container

```bash
docker run -it --runtime=runc \
    --interactive --env=DISPLAY=$DISPLAY \
    --env=QT_X11_NO_MITSHM=1 \
    --volume=/tmp/.X11-unix:/tmp/.X11-unix:rw \
    --volume=$HOME/.ssh:/home/ros/.ssh \
    --name=caixinha_container \
    caixinha
```

On Windows, add this:

```bash
export DISPLAY=:0
```

To enter an already running container:

```bash
docker exec -it caixinha_container /bin/bash
```

## Run ROS 2 ArduPilot Tests

```bash
cd ~/ardu_ws/
colcon test --packages-select ardupilot_dds_tests
```

## Build ROS 2 Packages

```bash
cd ~/ardu_ws
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
cd ~/ardu_ws/
source install/setup.bash
ros2 launch ardupilot_gz_bringup iris_runway.launch.py
```

### Test QGroundControl

```bash
cd ~
```bash
# Run QGroundControl directly from the AppImage
./QGroundControl.AppImage
```
# Extract QGroundControl AppImage
./QGroundControl.AppImage --appimage-extract
./squashfs-root/QGroundControl
```

## Additional Commands

Refer to [commands.txt](commands.txt) and [tests.txt](tests.txt) for more setup details.

## Contributions

Contributions are welcome! Feel free to open an issue or submit a pull request.


