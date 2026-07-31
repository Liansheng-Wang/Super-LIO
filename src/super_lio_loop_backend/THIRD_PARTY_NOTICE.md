# Third-party notice

This package contains adapted source code from:

- Project: **Voxel-SLAM**
- Upstream: https://github.com/hku-mars/Voxel-SLAM
- Revision: `70fc8a28d63823d5989ff184daeea0787b672398`
- Upstream license: GNU General Public License version 2 only

The upstream license text is preserved in `LICENSE`.

Adapted files:

- `VoxelSLAM/src/BTC.h` -> `include/super_lio_loop_backend/upstream/BTC.h`
- `VoxelSLAM/src/BTC.cpp` -> `src/BTC.cpp`
- `VoxelSLAM/src/tools.hpp` -> `include/super_lio_loop_backend/upstream/tools.hpp`
- `VoxelSLAM/src/preintegration.hpp` -> `include/super_lio_loop_backend/upstream/preintegration.hpp`
- `VoxelSLAM/src/voxel_map.hpp` -> `include/super_lio_loop_backend/upstream/voxel_map.hpp`
- `VoxelSLAM/src/loop_refine.hpp` -> `include/super_lio_loop_backend/upstream/loop_refine.hpp`

The adaptations remove ROS1-specific includes and parameter loading, use ROS 2
message types where a declaration remains necessary, and isolate the original
BTC, planar factor, and OctreeGBA algorithms inside a standalone GPLv2 process.
Voxel-SLAM's IMU frontend and multi-session graph logic are not used.
