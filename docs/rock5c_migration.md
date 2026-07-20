# Rock5C Ubuntu 24.04 / ROS 2 Jazzy Migration

This document describes how to migrate the four local ROS 2 packages to a
Rock5C running a Rockchip Ubuntu 24.04 image.

The four repositories are:

- `ros2_astra_camera_Jazzy`: Orbbec/Astra camera driver and messages.
- `serial_port_test`: chassis serial driver, odometry, IMU, battery, and `/cmd_vel`.
- `cspc_lidar_ros2_Jazzy`: CSPC lidar driver and scan publishing.
- `robot_phase1_bringup`: phase 1 lidar SLAM and Nav2 bringup/config package.

Do not copy `build/`, `install/`, or `log/` directories from the VM. The Rock5C
is `aarch64`, so every workspace must be rebuilt on the board.

## 1. Confirm the Rock5C Platform

Run on the board:

```bash
uname -m
lsb_release -a
```

Expected:

- `uname -m`: `aarch64`
- Ubuntu codename: `noble`

## 2. Recommended One-Shot Setup

Install minimal GitHub tooling, log in, then clone this bringup repository so the setup script is available:

```bash
sudo apt update
sudo apt install -y git gh

gh auth login --web --git-protocol https --insecure-storage
gh auth setup-git

mkdir -p ~/robot_storage/phase1_nav_ws/src
git clone https://github.com/kuonishenshang/robot_phase1_bringup.git \
  ~/robot_storage/phase1_nav_ws/src/robot_phase1_bringup
```

Run the setup script:

```bash
bash ~/robot_storage/phase1_nav_ws/src/robot_phase1_bringup/scripts/setup_rock5c_jazzy.sh
```

The script installs ROS 2 Jazzy dependencies, clones or updates the four
repositories, installs hardware permissions and stable serial-device aliases,
builds the workspaces in dependency order, and writes:

```bash
~/robot_storage/setup_phase1.bash
```

Useful script switches:

```bash
# Skip apt installation and only clone/build.
INSTALL_ROS=0 bash scripts/setup_rock5c_jazzy.sh

# Skip camera build if phase 1 lidar SLAM/Nav2 is the only target.
BUILD_CAMERA=0 bash scripts/setup_rock5c_jazzy.sh

# Avoid installing RViz packages on a headless board.
INSTALL_RVIZ=0 bash scripts/setup_rock5c_jazzy.sh

# Skip user/group and udev changes during a dry run.
INSTALL_HARDWARE_PERMISSIONS=0 bash scripts/setup_rock5c_jazzy.sh

# Continue on a non-Rock5C/non-aarch64 test machine.
ALLOW_UNSUPPORTED=1 bash scripts/setup_rock5c_jazzy.sh

# Use rosdepc after installing it as described below.
ROSDEP_COMMAND=rosdepc bash scripts/setup_rock5c_jazzy.sh

# Skip dependency metadata checks when dependencies were already installed.
RUN_ROSDEP=0 bash scripts/setup_rock5c_jazzy.sh
```

### Recover From a rosdep Download Failure

`rosdep init` and `rosdep update` download metadata from GitHub Raw. A
`website may be down` error usually means that service is not reachable from
the current network; it does not mean the local ROS 2 installation is damaged.

On networks where GitHub Raw is unavailable, install the third-party `rosdepc`
mirror client in an isolated Python environment:

```bash
sudo apt update
sudo apt install -y pipx
pipx install rosdepc

ROSDEPC="${HOME}/.local/bin/rosdepc"
sudo "${ROSDEPC}" init
"${ROSDEPC}" update
```

If the source list already exists and still refers to
`raw.githubusercontent.com`, back it up before running `rosdepc init`:

```bash
sudo mv \
  /etc/ros/rosdep/sources.list.d/20-default.list \
  /etc/ros/rosdep/sources.list.d/20-default.list.official
sudo "${HOME}/.local/bin/rosdepc" init
"${HOME}/.local/bin/rosdepc" update
```

Then update this repository and resume without repeating apt installation:

```bash
cd ~/robot_storage/phase1_nav_ws/src/robot_phase1_bringup
git pull --ff-only

INSTALL_ROS=0 ROSDEP_COMMAND="${HOME}/.local/bin/rosdepc" \
  bash scripts/setup_rock5c_jazzy.sh
```

If both metadata services are unavailable, use `RUN_ROSDEP=0`. The setup
script already installs the dependencies currently required by all four
repositories explicitly.

## 3. Manual Setup Commands

Install ROS 2 Jazzy and dependencies:

```bash
sudo apt update
sudo apt install -y software-properties-common curl gnupg lsb-release ca-certificates
sudo add-apt-repository -y universe

export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

sudo apt update
sudo apt install -y \
  ros-jazzy-ros-base ros-dev-tools \
  python3-colcon-common-extensions python3-rosdep python3-vcstool \
  build-essential cmake git gh pkg-config \
  libboost-system-dev libpcl-dev libuvc-dev libgoogle-glog-dev libgflags-dev libusb-1.0-0-dev libeigen3-dev \
  ros-jazzy-slam-toolbox ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
  ros-jazzy-pcl-ros ros-jazzy-pcl-conversions \
  ros-jazzy-cv-bridge ros-jazzy-image-geometry ros-jazzy-camera-info-manager \
  ros-jazzy-image-transport ros-jazzy-image-publisher ros-jazzy-tf2-sensor-msgs ros-jazzy-tf2-eigen \
  ros-jazzy-rviz2

sudo rosdep init || true
rosdep update
```

Clone with `vcs`:

```bash
mkdir -p ~/robot_storage
vcs import ~/robot_storage < ~/robot_storage/phase1_nav_ws/src/robot_phase1_bringup/repos/rock5c_phase1.repos
```

Or clone manually:

```bash
mkdir -p ~/robot_storage/camera_test_ws/src
mkdir -p ~/robot_storage/serial_test_ws/src
mkdir -p ~/robot_storage/lidar_test_ws/src
mkdir -p ~/robot_storage/phase1_nav_ws/src

git clone https://github.com/kuonishenshang/ros2_astra_camera_Jazzy.git ~/robot_storage/camera_test_ws/src/ros2_astra_camera
git clone https://github.com/kuonishenshang/serial_port_test.git ~/robot_storage/serial_test_ws/src/serial_port_test
git clone https://github.com/kuonishenshang/cspc_lidar_ros2_Jazzy.git ~/robot_storage/lidar_test_ws/src/cspc_lidar_sdk_ros2
git clone https://github.com/kuonishenshang/robot_phase1_bringup.git ~/robot_storage/phase1_nav_ws/src/robot_phase1_bringup
```

Build in dependency order:

```bash
source /opt/ros/jazzy/setup.bash

cd ~/robot_storage/serial_test_ws
colcon build --symlink-install

source ~/robot_storage/serial_test_ws/install/setup.bash
cd ~/robot_storage/lidar_test_ws
colcon build --symlink-install

source ~/robot_storage/lidar_test_ws/install/setup.bash
cd ~/robot_storage/phase1_nav_ws
colcon build --symlink-install

source /opt/ros/jazzy/setup.bash
cd ~/robot_storage/camera_test_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## 4. Hardware Permissions

Run once, then reboot:

```bash
sudo usermod -aG dialout,video,plugdev $USER

sudo install -m 0644 \
  ~/robot_storage/phase1_nav_ws/src/robot_phase1_bringup/udev/99-robot-phase1.rules \
  /etc/udev/rules.d/99-robot-phase1.rules

cd ~/robot_storage/camera_test_ws/src/ros2_astra_camera/astra_camera/scripts
sudo bash install.sh
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo udevadm settle
sudo reboot
```

After reboot, check device names:

```bash
ls -l /dev/robot_base /dev/robot_lidar
```

The default launch values are:

- chassis: `/dev/robot_base`, `230400`
- lidar: `/dev/robot_lidar`, `230400`

The chassis alias is matched by USB serial number `5B0A017238`. The lidar's
CH341 adapter has no unique serial number, so `robot_lidar` is matched to the
Rock5C physical USB path `platform-fc8c0000.usb-usb-0:1:1.0`. Keep the lidar
connected to that port. The launch arguments remain available for diagnostics
or replacement hardware.

## 5. Runtime Verification

Source the environment:

```bash
source ~/robot_storage/setup_phase1.bash
```

Check package discovery:

```bash
ros2 pkg prefix serial_port_test
ros2 pkg prefix cspc_lidar
ros2 pkg prefix robot_phase1_bringup
```

Start mapping on the board without RViz:

```bash
ros2 launch robot_phase1_bringup mapping.launch.py start_rviz:=false
```

Check lidar and TF:

```bash
ros2 topic hz /scan
ros2 run tf2_ros tf2_echo base_link laser_link
```

Save the map:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/robot_storage/phase1_nav_ws/src/robot_phase1_bringup/maps/map
```

Start navigation:

```bash
ros2 launch robot_phase1_bringup navigation.launch.py \
  start_rviz:=false \
  map:=~/robot_storage/phase1_nav_ws/src/robot_phase1_bringup/maps/map.yaml
```

If required:

```bash
ros2 launch robot_phase1_bringup mapping.launch.py \
  start_rviz:=false \
  base_serial_port:=/dev/ttyACM0 \
  lidar_port:=/dev/ttyUSB0
```

## 6. Topic and TF Contract

The phase 1 bringup assumes:

- `/cmd_vel`: Nav2 velocity command to chassis.
- `/odom`: chassis odometry.
- `/imu/data`: chassis IMU.
- `/battery/voltage`: chassis battery voltage.
- `/scan`: lidar LaserScan.
- `map -> odom`: SLAM Toolbox or AMCL.
- `odom -> base_link`: chassis node.
- `base_link -> laser_link`: static transform from `robot_phase1_bringup`.
