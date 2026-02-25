# AGENTS.md

## Cursor Cloud specific instructions

This is a ROS 2 C++ package (`rs_to_velodyne`) that converts Robosense LiDAR point clouds to Velodyne format.

### Environment

- **OS**: Ubuntu 24.04 (Noble)
- **ROS 2 distro**: Jazzy Jalisco (`/opt/ros/jazzy/`)
- **Build system**: `colcon` (ament_cmake)
- **C++ standard**: C++14

### Compiler gotcha

The default `c++`/`cc` alternatives must point to `g++`/`gcc` (not `clang++`/`clang`), otherwise the build fails with `-lstdc++ not found`. The update script handles this automatically:

```bash
sudo update-alternatives --set cc /usr/bin/gcc
sudo update-alternatives --set c++ /usr/bin/g++
```

### Build

```bash
source /opt/ros/jazzy/setup.bash
cd /workspace
colcon build
```

The CMake warning about `CMP0144` / `FLANN_ROOT` is harmless and comes from system PCL; ignore it.

### Run

After building, source both setups then run:

```bash
source /opt/ros/jazzy/setup.bash
source /workspace/install/setup.bash
ros2 run rs_to_velodyne rs_to_velodyne <INPUT_TYPE> <OUTPUT_TYPE>
```

Where `<INPUT_TYPE>` is `XYZI` or `XYZIRT`, and `<OUTPUT_TYPE>` is `XYZI`, `XYZIR`, or `XYZIRT`. See `README.md` for details.

### Testing

There are no automated tests in the repository. To verify the node works, start it and publish a `sensor_msgs/PointCloud2` on `/rslidar_points`; check for output on `/velodyne_points`.

### Lint

No lint configuration is included. For ROS 2 linting, `ament_lint_auto` can be added to `CMakeLists.txt` if needed.
