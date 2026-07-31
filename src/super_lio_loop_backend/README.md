# super_lio_loop_backend

Independent GPLv2 ROS 2 process providing:

- BTC/STD triangle-descriptor loop detection;
- a single-session GTSAM ISAM2 pose graph;
- an on-demand Voxel-SLAM `OctreeGBA` planar global bundle adjustment.

Run:

```bash
ros2 launch super_lio_loop_backend loop_backend.py
ros2 service call /super_lio_loop_backend/run_global_ba \
  super_lio_loop_msgs/srv/RunGlobalBA '{}'
```

Corrected keyframe poses are available from
`/super_lio_loop_backend/get_corrected_poses`.

GTSAM is a non-ament dependency and is intentionally discovered with standard
`find_package(GTSAM REQUIRED)`.
