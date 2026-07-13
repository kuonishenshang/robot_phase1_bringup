# Saved Maps

This directory is kept in the repository so a fresh checkout can be installed
before the first SLAM map is saved.

Save a map after mapping with:

```bash
ros2 run nav2_map_server map_saver_cli -f \
  ~/robot_storage/phase1_nav_ws/src/robot_phase1_bringup/maps/map
```
