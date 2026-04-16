# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build Commands

### Full build (Thirdparty + ORB-SLAM3)
```bash
./build.sh
```
This builds DBoW2, g2o, Sophus (in `Thirdparty/`), extracts the ORB vocabulary, then builds the main library and all examples.

### Build only ORB-SLAM3 (after Thirdparty is built)
```bash
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j3
```
Output: `lib/libORB_SLAM3.so` and executables in `Examples/` subdirectories.

### Build ROS nodes (optional)
```bash
# First add to ~/.bashrc: export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:$(pwd)/Examples/ROS
./build_ros.sh
```

### Rebuild a single example
```bash
cd build && make stereo_euroc -j3
```

## Running Examples

General pattern: `<executable> Vocabulary/ORBvoc.txt <config.yaml> [dataset args]`

**EuRoC stereo-inertial:**
```bash
./Examples/Stereo-Inertial/stereo_inertial_euroc Vocabulary/ORBvoc.txt Examples/Stereo-Inertial/EuRoC.yaml /path/to/dataset MH_01_easy ./Examples/Stereo-Inertial/EuRoC_TimeStamps/MH01.txt
```

**ROS stereo-inertial:**
```bash
source Examples/ROS/ORB_SLAM3/build/devel/setup.bash
rosrun ORB_SLAM3 Stereo_Inertial Vocabulary/ORBvoc.txt Examples/Stereo-Inertial/EuRoC.yaml true
rosbag play --pause V1_02_medium.bag /cam0/image_raw:=/camera/left/image_raw /cam1/image_raw:=/camera/right/image_raw /imu0:=/imu
```

**RGB-D with ROS:**
```bash
rosrun ORB_SLAM3 RGBD Vocabulary/ORBvoc.txt Examples/RGB-D/TUM1.yaml
rosbag play rgbd_dataset_freiburg1_xyz.bag /camera/rgb/image_color:=/camera/rgb/image_raw /camera/depth/image:=/camera/depth_registered/image_raw
```

## Enabling Timing Measurements

In `include/Config.h`, uncomment `#define REGISTER_TIMES` to collect per-frame timing stats. Results are printed to terminal and saved to `ExecTimeMean.txt`.

## Architecture Overview

ORB-SLAM3 is a multi-threaded SLAM library compiled as a shared library (`libORB_SLAM3.so`). Entry point for all usage is `ORB_SLAM3::System` (`include/System.h`, `src/System.cc`).

### Threading Model

`System` spawns three parallel threads:
- **Tracking** (`src/Tracking.cc`) — main thread; called per-frame via `TrackMonocular`, `TrackStereo`, or `TrackRGBD`; extracts ORB features, estimates camera pose
- **LocalMapping** (`src/LocalMapping.cc`) — receives keyframes from Tracking; triangulates new MapPoints, runs local bundle adjustment
- **LoopClosing** (`src/LoopClosing.cc`) — detects loop closures via DBoW2; triggers map merging and global bundle adjustment

### Key Data Structures

- **`Frame`** — a single camera observation with extracted keypoints and descriptors
- **`KeyFrame`** — a selected Frame retained in the map; linked in a covisibility graph
- **`MapPoint`** — a 3D landmark observed from multiple KeyFrames
- **`Map`** — a collection of KeyFrames and MapPoints; one active map at a time
- **`Atlas`** (`src/Atlas.cc`, `include/Atlas.h`) — manages the multi-map system; holds all Maps (active + stored); supports boost serialization for save/load

### Feature Extraction and Matching

- **`ORBextractor`** (`src/ORBextractor.cc`) — modified from OpenCV; extracts ORB features with pyramid scale
- **`ORBmatcher`** (`src/ORBmatcher.cc`) — matches descriptors between frames and map points using Hamming distance
- **`KeyFrameDatabase`** (`src/KeyFrameDatabase.cc`) — inverted index over DBoW2 vocabulary for place recognition

### Camera Models

Located in `src/CameraModels/` and `include/CameraModels/`:
- **`Pinhole`** — standard pin-hole with distortion
- **`KannalaBrandt8`** — fisheye model (used for TUM-VI dataset)
- Both derive from `GeometricCamera` abstract interface

### Optimization

- **`Optimizer`** (`src/Optimizer.cc`) — wraps g2o; implements pose-only BA, local BA, full BA, IMU initialization, and Sim3 optimization for loop closing
- **`G2oTypes`** / **`OptimizableTypes`** — custom g2o vertex/edge types for SE3, Sim3, IMU preintegration

### IMU Integration

- **`ImuTypes`** (`src/ImuTypes.cc`) — IMU measurement and preintegration; `IMU::Preintegrated` accumulates bias-corrected delta pose between keyframes
- IMU data is passed alongside images via optional `vImuMeas` argument in the Track* methods

### Settings and Vocabulary

- **`Settings`** (`src/Settings.cc`) — parses YAML config files (camera intrinsics, ORB parameters, IMU noise, viewer settings)
- **`ORBVocabulary`** (`include/ORBVocabulary.h`) — DBoW2 vocabulary loaded at startup from `Vocabulary/ORBvoc.txt` (extracted from `ORBvoc.txt.tar.gz` by `build.sh`)

### Thirdparty Libraries (in `Thirdparty/`)

- **DBoW2** — bag-of-words place recognition
- **g2o** — graph optimization
- **Sophus** — Lie group/algebra (SE3, Sim3) used throughout via `Sophus::SE3f`

### Examples vs Examples_old

`Examples/` contains the current API using the new `Settings`-based YAML parser. `Examples_old/` uses the older direct `cv::FileStorage` parsing pattern. Prefer `Examples/` for new work.
