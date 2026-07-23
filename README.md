# Phase 1 SLAM and Navigation Bringup

This workspace is an overlay bringup layer. It does not copy or modify the
validated chassis, lidar, and Astra camera packages.

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

The setup script installs stable serial-device aliases:

- `/dev/robot_base`: chassis controller matched by its unique USB serial number.
- `/dev/robot_lidar`: lidar matched by its physical USB port because its CH341
  adapter does not expose a unique serial number.

Keep the lidar connected to the configured Rock5C USB port.

## Build

Source the already-built lower workspaces before building this overlay:

```bash
source /opt/ros/jazzy/setup.bash
source /home/dzx/robot_storage/serial_test_ws/install/setup.bash
source /home/dzx/robot_storage/lidar_test_ws/install/setup.bash
source /home/dzx/robot_storage/camera_test_ws/install/setup.bash
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

The navigation launch starts the Astra Pro RGB, depth, and XYZ point-cloud
streams by default. Disable the camera for lidar-only operation with:

```bash
ros2 launch robot_phase1_bringup navigation.launch.py start_camera:=false
```

The tested Astra Pro profile is RGB `640x480@15` and depth `640x480@30`.
The device does not support depth `640x480@15` and otherwise falls back to
30 Hz with a driver warning.

The camera mount is published as `base_link -> camera_link` with a default
translation of `(0.08, 0.0, 0.0)` metres and an identity rotation. The depth
point cloud feeds a local-costmap VoxelLayer; voxel debug output remains
enabled during Phase 1 integration.

The VoxelLayer publishes `/local_costmap/voxel_grid`. A debug converter started
by this launch publishes its marked cells as the RViz-compatible
`/local_costmap/voxel_marked_cloud`. Disable it outside debugging with
`publish_voxel_debug:=false`.

## Interfaces

- `/cmd_vel`: chassis command input.
- `/odom`: chassis odometry.
- `/imu/data`: chassis IMU data.
- `/battery/voltage`: chassis battery voltage.
- `/scan`: lidar scan.
- `/camera/color/image_raw`: Astra Pro RGB image.
- `/camera/depth/image_raw`: Astra Pro depth image.
- `/camera/depth/points`: depth-only XYZ point cloud.
- `/local_costmap/voxel_grid`: raw Nav2 voxel-grid debug data.
- `/local_costmap/voxel_marked_cloud`: marked voxels for RViz display.
- TF: `map -> odom` from SLAM or AMCL, `odom -> base_link` from chassis,
  `base_link -> laser_link` and `base_link -> camera_link` from this bringup
  package, and camera-internal optical frames from the Astra driver.
