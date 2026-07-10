# Phase 1 Lidar SLAM and Navigation Bringup

This workspace is an overlay bringup layer. It does not copy or modify the
validated chassis and lidar packages.

## Required system packages

Install the runtime packages once:

```bash
sudo apt-get update
sudo apt-get install -y ros-jazzy-slam-toolbox ros-jazzy-navigation2 ros-jazzy-nav2-bringup
```

For Rock5C Ubuntu 24.04 migration, use the scripted setup:

```bash
bash ~/robot_storage/phase1_nav_ws/src/robot_phase1_bringup/scripts/setup_rock5c_jazzy.sh
```

See `docs/rock5c_migration.md` for the full board-side install, clone, build,
hardware-permission, and validation workflow.

## Build

Source the already-built lower workspaces before building this overlay:

```bash
source /opt/ros/jazzy/setup.bash
source /home/dzx/robot_storage/serial_test_ws/install/setup.bash
source /home/dzx/robot_storage/lidar_test_ws/install/setup.bash
cd /home/dzx/robot_storage/phase1_nav_ws
colcon build
source install/setup.bash
```

## Mapping

```bash
ros2 launch robot_phase1_bringup mapping.launch.py
```

Drive the robot manually with teleop on `/cmd_vel`. Save a finished map with:

```bash
ros2 run nav2_map_server map_saver_cli -f /home/dzx/robot_storage/phase1_nav_ws/src/robot_phase1_bringup/maps/map
```

When launching navigation, pass the saved source-path map explicitly, as shown below. Rebuild only if you want the default installed `maps/map.yaml` path to work.

## Navigation

```bash
ros2 launch robot_phase1_bringup navigation.launch.py \
  map:=/home/dzx/robot_storage/phase1_nav_ws/src/robot_phase1_bringup/maps/map.yaml
```

Use RViz to set the initial pose, wait for AMCL to converge, then send a 2D
navigation goal.

## Interfaces

- `/cmd_vel`: chassis command input.
- `/odom`: chassis odometry.
- `/imu/data`: chassis IMU data.
- `/battery/voltage`: chassis battery voltage.
- `/scan`: lidar scan.
- TF: `map -> odom` from SLAM or AMCL, `odom -> base_link` from chassis,
  `base_link -> laser_link` from this bringup package.
