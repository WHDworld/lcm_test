# LCM Test Project

## 项目概述
本项目 `lcm_test` 是一个基于 LCM（Lightweight Communications and Marshalling）的测试项目，主要用于无人机轨迹数据的广播和接收。通过 LCM 实现了数据的高效通信，利用 ROS 进行日志管理，并使用 C++ 语言编写。

## 依赖环境
- **LCM**：用于数据的通信和编组。
- **ROS**：用于日志管理和时间处理。
- **Eigen**：用于向量和四元数的处理。
- **CMake**：用于项目的构建。
## LCM库安装
```bash
sudo apt install openjdk-8-jdk
git clone -b v1.5.0 https://github.com/lcm-proj/lcm.git
cd lcm
mkdir build && cd build && \
      cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF -DBUILD_BENCHMARKS=OFF .. && \
      sudo make -j8 install
## 代码结构
```
lcm_test/
├── CMakeLists.txt
├── include/
│   ├── Point3d_t.hpp
│   ├── TrajectoryDescriptor_t.hpp
│   ├── IMUData_t.hpp
│   ├── Pose_t.hpp
│   ├── lcm_net.h
│   └── ...
├── src/
│   ├── main.cpp
│   └── lcm_net.cpp
└── lcm_generate_msg.sh
```

### 主要文件说明
- **CMakeLists.txt**：项目的构建配置文件，定义了项目的依赖和编译规则。
- **include/ 目录**：包含项目的头文件，定义了各种数据结构和类。
  - `Point3d_t.hpp`：定义了三维点的数据结构。
  - `TrajectoryDescriptor_t.hpp`：定义了轨迹描述符的数据结构。
  - `lcm_net.h`：定义了 LCM 网络通信的类和相关函数。
- **src/ 目录**：包含项目的源文件，实现了具体的功能。
  - `main.cpp`：项目的入口文件，初始化 LCM 网络并进行轨迹数据的广播。
  - `lcm_net.cpp`：实现了 LCM 网络通信的具体功能，包括数据的发送和接收。
- **lcm_generate_msg.sh**：用于生成 LCM 消息的脚本。

## 编译和运行步骤

### 编译项目
1. 确保已经安装了所需的依赖环境。
2. 创建一个工作空间并进入该空间：
```bash
mkdir -p catkin_ws/src
cd catkin_ws/src
```
3. 将项目克隆到 `src` 目录下：
```bash
git clone <项目仓库地址> lcm_test
```
4. 进入工作空间的根目录并编译项目：
```bash
cd ..
catkin_make
```

### 运行项目
1. 激活工作空间：
```bash
source devel/setup.bash
```
2. 运行项目：
```bash
rosrun lcm_test lcm_test
```

## 功能说明

### 数据结构
- **`Point3d_t`**：表示三维空间中的一个点，包含 `x`、`y`、`z` 三个坐标。
- **`TrajectoryDescriptor_t`**：表示无人机的轨迹描述符，包含时间戳、自身 ID 和自身姿态。
- **`IMUData_t`**：表示惯性测量单元（IMU）的数据，包含时间戳、时间间隔、加速度和角速度。
- **`Pose_t`**：表示无人机的姿态，包含位置、速度、加速度和方向。
- **`Trajectory_t`**：表示无人机的轨迹，包含自身 ID、时间戳、自身姿态、速度、加速度和方向。

### 网络通信
- **`LcmNet` 类**：实现了 LCM 网络通信的功能，包括数据的广播和接收。
  - `broadcastTrajectory` 方法：将 `Trajectory_t` 类型的轨迹数据转换为 `TrajectoryDescriptor_t` 类型，并通过 LCM 广播出去。
  - `onTrajectoryReceived` 方法：处理接收到的轨迹数据，当数据的自身 ID 与本地 ID 不同时，调用回调函数进行处理。
### 自定义数据结构
- **`SwarmPose_t.lcm` **：在SwarmPose_t.lcm中添加你自定义的数据结构。
  - `broadcastTrajectory` 方法：将 `Trajectory_t` 类型的轨迹数据转换为 `TrajectoryDescriptor_t` 类型，并通过 LCM 广播出去。
  - `onTrajectoryReceived` 方法：处理接收到的轨迹数据，当数据的自身 ID 与本地 ID 不同时，调用回调函数进行处理。
## 注意事项
- 请确保 LCM 的 URI 配置正确，以保证数据的正常通信。
- 在编译和运行项目时，需要激活 ROS 工作空间。

## 贡献和反馈
如果您对本项目有任何建议或发现了问题，请在项目的 GitHub 仓库中提交 issue 或 pull request。
