# 部署指南 / Deployment Guide

## 环境要求

- Ubuntu 20.04 LTS (x86_64)
- ROS Noetic
- Qt 5.12+
- GCC 9+ (支持 C++17)

## 1. 系统依赖安装

```bash
# 基础编译工具
sudo apt install build-essential cmake git

# Qt 5
sudo apt install qt5-default qtcreator

# OpenCV 4
sudo apt install libopencv-dev

# ROS 相关
sudo apt install ros-noetic-roscpp ros-noetic-geometry-msgs \
  ros-noetic-tf2 ros-noetic-tf2-ros ros-noetic-actionlib \
  ros-noetic-move-base-msgs ros-noetic-nav-msgs

# CUDA（可选，用于 ZED SDK 加速）
# 从 NVIDIA 官网下载安装 CUDA 11.x
```

## 2. ZED SDK 安装

从 [Stereolabs 官网](https://www.stereolabs.com/developers/release/) 下载 ZED SDK for Ubuntu 20.04，按官方文档安装。

```bash
# 验证安装
/usr/local/zed/tools/ZED_Diagnostic
```

## 3. AUBO Robot SDK 配置

1. 从遨博官方获取 Linux C++ SDK
2. 解压到项目目录：
```bash
mkdir -p src/dependents/robotSDK
# 将 SDK 的 inc/ 和 lib/ 复制到 src/dependents/robotSDK/
```

3. 配置 log4cplus：
```bash
mkdir -p src/dependents/log4cplus
# 将 log4cplus 库复制到对应目录
```

## 4. USB-CAN 驱动

```bash
# 将 libusbcanfd.so 放入 src/lib/
# 或将 libusb-1.0 通过 apt 安装
sudo apt install libusb-1.0-0-dev
```

## 5. ROS 工作空间配置

```bash
# 创建 ROS 工作空间
mkdir -p ~/ws_sentry/src
cd ~/ws_sentry/src

# 克隆/放置以下 ROS 包：
# - livox_ros_driver2      (Livox 激光雷达驱动)
# - fast_lio_localization  (Fast-LIO 定位)
# - sentry_nav             (move_base 导航配置)
# - agv_navigation         (AGV 导航任务)

# 编译
cd ~/ws_sentry
catkin_make

# 设置环境变量
echo "source ~/ws_sentry/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 6. 编译上位机

```bash
cd src
qmake AGV_test.pro
make -j$(nproc)
```

## 7. 运行

```bash
# 设置 ROS 工作空间（如未在 .bashrc 中设置）
export ROS_WORKSPACE=~/ws_sentry

# 启动上位机
./AGV_test
```

## 8. 操作流程

1. 点击 **启动雷达** → 等待 Livox 驱动就绪
2. 点击 **启动定位** → 等待 Fast-LIO 初始化
3. 点击 **发布初始位姿** → 设置机器人起点
4. 点击 **movebase** → 启动路径规划
5. 点击 **AGV** → 启动导航执行
6. 登录机械臂 → 上电
7. 开始巡检任务

## 故障排查

| 问题 | 可能原因 | 解决方法 |
|------|----------|----------|
| 串口无法打开 | 权限不足 | `sudo usermod -aG dialout $USER` |
| ROS 进程启动失败 | 工作空间未 source | 确认 `ROS_WORKSPACE` 环境变量 |
| ZED 相机打不开 | USB 3.0 带宽不足 | 检查 USB 口，降低分辨率 |
| CAN 通信失败 | 驱动未加载 | 重新插拔 USB-CAN 适配器 |
