# person_target_fusion

This package associates the highest-confidence valid YOLO `person` observation with an IWR6843 Long Range People Tracking track.

The node transforms both inputs to `odom` at their measurement timestamps. Radar velocity output is disabled by default until mobile-platform ego-motion compensation has been validated. A radar track is locked only after `association_confirm_frames` distinct radar frame numbers have passed the position gate. Once locked, the lock is never replaced by another radar ID during the same target session. While the locked track is visible to the camera, the output position comes from vision; when the camera observation times out, a fresh observation from the locked radar ID can take over. When both observations expire the node publishes `LOST` and requires a new visual confirmation.

## Inputs

- `/yolo/detections`: `rknn_yolov8_ros/msg/DetectionArray`
- `/radar/tracks`: `person_tracking_msgs/msg/RadarTrackArray`

The radar driver is intentionally outside this package. It must publish a measurement timestamp, a ROS REP-103 frame, and a frame number that is monotonic during a tracker session.

## Output

`/person_tracking/target` (`person_tracking_msgs/msg/FusedPersonTarget`) is always expressed in `odom`. `navigation_allowed` is false for `SENSOR_GAP` and `LOST`; downstream navigation must cancel or stop accepting new goals in those states.

## Standalone launch

```bash
ros2 launch person_target_fusion person_target_fusion.launch.py \
  vision_topic:=/yolo/detections radar_topic:=/radar/tracks
```

For phase1 navigation, set `start_person_target_fusion:=true` and provide the radar topic. The current `person_following` executable still consumes the legacy YOLO topic; refactoring it to consume `FusedPersonTarget` is a separate integration step.
