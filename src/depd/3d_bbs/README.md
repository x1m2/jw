<div align="center">
  <h1>3D-BBS: Global Localization for 3D Point Cloud Scan Matching Using Branch-and-Bound Algorithm</h1>
  <br>
  <p>This repository contains the public source code for the paper to be published at ICRA2024 in Yokohama.</p>
  <p>If you like the 3D-BBS, we kindly ask for your support by giving our repository star ⭐!</p>
  <p><a href="https://arxiv.org/abs/2310.10023"><b>📄  Paper</b></a> | <a href="https://youtu.be/QGxnDnwkG1k"><b>🎞️ Video</b></a></p>
</div>

```
@inproceedings{aoki20243dbbs,
  title={3D-BBS: Global Localization for 3D Point Cloud Scan Matching Using Branch-and-Bound Algorithm}, 
  author={Koki Aoki and Kenji Koide and Shuji Oishi and Masashi Yokozuka and Atsuhiko Banno and Junichi Meguro},
  booktitle={IEEE International Conference on Robotics and Automation},
  year={2024},
  organization={IEEE}
}
```

## 🔍 Quick understanding about 3D-BBS
<img alt="overview" src="figs/overview.jpg" width="50%">

### Task
- 3D global localization for such as robot's initial pose estimation

### Input
  - A single 3D LiDAR scan aligned in the gravity direction by using such as IMU   
  (Although 3D-BBS also performs 6DoF search without gravity aligning, the processing time is more than 10 times longer.)
  - 3D pre-built point cloud map

### Output
- $4\times4$ transformation matrix about coarse sensor pose in map coordinates.

### Main Features 
- **Full search algorithm** based on branch-and-bound (BnB) scan matching
- **Fast processing (latest: about 20 ms ~)** by proposed GPU-accelerated BnB and roto-translational branching
- **No initial pose** (we assume that the roll and pitch angles are roughly estimated by IMU for faster processing)
- **Low voxelmap memory consumption** due to sparse hash map

### Other Features
<details><summary> Click here! </summary><div>

- Optional **Voxelmap pre-construction** and direct read function
- Optional **timeout** function
- **Minimal dependencies** in core source code so that user can **easily integrate** into the other projects
- **Docker support** by @Taeyoung96
- **Test data** is available, allowing evaluation of localization error and processing time.
- **ROS2 implementation example**
</div></details>

## 📣 News
The latest implementation demonstrates faster processing time than our paper. 
Performance on our test data:
- Hierarchical voxelmap construction
  - Paper: 9,272 ms on average
  - **Latest**: 3,494 ms on average
  - **Load saved voxelmaps directly**: 130 ms on average (See [step 5 on this page](./test/test_code.md) to save voxelmaps)
- Global localization
  - Paper: 878 ms on average
  - **Latest**: **189 ms** on average  

The hardware configuration (Intel Core i7-10700K 3.8GHz, 32GB RAM, and NVIDIA GeForce RTX2060) is the same as in the paper.

## ✅ Dependencies
- bbs3d (Lower versions are not tested)
  - CMake
  - Eigen3 (3.4.0 or higher)
  - OpenMP
  - CUDA (12.0 or higher)
- test
  - (All bbs3d dependencies)
  - PCL
- ros2_test
  - (All bbs3d dependencies)
  - ros2 humble

### Support Docker 🐳 

If nvidia driver is 525.60.11 or higher, try docker!  
For more information, you can check [docker_start.md](./docker/docker_start.md)  

## 🌟 3D-BBS core source code
### Build and Install

在工作区根目录选择一种后端构建；两个脚本都生成 `voxelmaps_saver`，不需要再进入 `test` 单独编译：

```bash
# CPU 板子：只生成 CPU 库和地图转换工具，不依赖 OpenCL。
./src/depd/3d_bbs/build_cpu.sh
colcon build --packages-select fastlio_localization --cmake-args -DENABLE_OPENCL_BBS=OFF

# OpenCL 板子：生成配套的 CPU/OpenCL 库和地图转换工具。
./src/depd/3d_bbs/build_opencl.sh
colcon build --packages-select fastlio_localization --cmake-args -DENABLE_OPENCL_BBS=ON

./install/bbs3d/bin/voxelmaps_saver /path/to/pcd_directory 1.0 6
```

默认安装到工作区 `install/bbs3d`，包含 `lib/libcpu_bbs3d.so`、
`bin/voxelmaps_saver` 和 CMake 包配置；OpenCL 构建额外包含
`lib/libopencl_bbs3d.so`。定位包通过 `find_package(bbs3d CONFIG)` 链接这些库，
不再重复编译或安装 BBS 库。两个脚本使用独立构建目录 `build_cpu/` 和 `build_opencl/`，
不会编译 `opencl_test`，无需 sudo。CPU 依赖 Eigen3、OpenMP、Boost；OpenCL
额外需要 OpenCL 开发库和板端 GPU 驱动。运行时分别设置 `bbs3d_backend: cpu/opencl`。

地图生成仍在 CPU 执行，CPU/OpenCL 共用地图格式，分辨率和层级应与定位配置一致。
可设置 `BUILD_JOBS`、`BBS_BUILD_DIR`、`BBS_INSTALL_DIR`；自定义安装位置时，
定位编译加 `-DBBS3D_ROOT=/安装目录`，OTA 更新设置同一个 `BBS_INSTALL_DIR`。
CPU/OpenCL 构建切换时如需保留两套产物，应分别指定不同安装目录。
额外参数传给 CMake；脚本固定所选后端和工具开关。

```shell script
# Note: If you are using Eigen3 below 3.4.0, git clone with --recursive
git clone https://github.com/KOKIAOKI/3d_bbs.git
cd 3d_bbs
mkdir build && cd build
```

- CPU ver. & GPU ver. (Please ignore the large number of warnings)
```shell script
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j
sudo make install
```

- CPU ver. only
```shell script
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_OPENCL=OFF
make -j
sudo make install
```

## 📋 Test code
See [test_code.md](./test/test_code.md)  
<img alt="overview" src="figs/test_topreadme.png" width="40%">

## 🤖 ROS 2 test code
See [ros2_test_code.md](./ros2_test/ros2_test_code.md)  
<img alt="overview" src="figs/rviz2_test_topreadme.png" width="40%">


## 💻 Use 3D-BBS in your cmake project
<details><summary> Click here! </summary><div>

1. Copy `test/cmake` to your project directory
1. Copy description above `# Common include directories` in `test/CMakeLists.txt` to `your CMakeLists.txt`
1. Add either of the following depending on your implementation  
If you use the CPU version, replace `gpu` with `cpu`.
```
find_package(Eigen3 REQUIRED)
target_include_directories(yours ${EIGEN3_INCLUDE_DIR} ${gpu_bbs3d_LIBRARY})
```
</div></details>

## 🌈 Acknowledgement
I sincerely appreciate the authors for generously sharing their public code.  
[hdl_global_localization](https://github.com/koide3/hdl_global_localization)  
[cartographer](https://github.com/cartographer-project/cartographer)  
[TEASER-plusplus](https://github.com/MIT-SPARK/TEASER-plusplus)   
